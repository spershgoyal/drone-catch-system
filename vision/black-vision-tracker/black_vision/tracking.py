from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import cv2
import numpy as np

from .detector import BlackColorModel, DetectionResult
from .tracker_config import CameraConfig, TrackerConfig
from .uwb import AnchorMeasurement


@dataclass(slots=True)
class BearingObservation:
    pixel_x: float
    pixel_y: float
    azimuth_rad: float
    elevation_rad: float
    direction_arm: np.ndarray
    confidence: float
    timestamp_s: float


@dataclass(slots=True)
class PositionEstimate:
    position_m: tuple[float, float, float]
    method: str
    residual: float
    range_count: int
    used_camera: bool
    distance_to_gripper_m: float
    capture_ready: bool


@dataclass(slots=True)
class TrackingFrameResult:
    detection: DetectionResult
    bearing: BearingObservation | None
    estimate: PositionEstimate | None
    measurements: list[AnchorMeasurement]

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "detected": self.detection.detected,
            "black_ratio": round(self.detection.black_ratio, 6),
            "confidence": round(self.detection.confidence, 6),
            "target_center": (
                None
                if self.detection.target_center is None
                else [
                    round(self.detection.target_center[0], 3),
                    round(self.detection.target_center[1], 3),
                ]
            ),
            "ranges_m": {
                measurement.anchor_id: round(measurement.distance_m, 4)
                for measurement in self.measurements
            },
        }
        if self.bearing is not None:
            payload["bearing_deg"] = {
                "azimuth": round(math.degrees(self.bearing.azimuth_rad), 3),
                "elevation": round(math.degrees(self.bearing.elevation_rad), 3),
            }
        if self.estimate is not None:
            payload["position_m"] = [round(value, 4) for value in self.estimate.position_m]
            payload["distance_to_gripper_m"] = round(
                self.estimate.distance_to_gripper_m,
                4,
            )
            payload["capture_ready"] = self.estimate.capture_ready
            payload["method"] = self.estimate.method
            payload["residual"] = round(self.estimate.residual, 6)
        return payload


class CameraModel:
    """Convert pixel centers into a 3D bearing in the arm coordinate frame."""

    def __init__(self, config: CameraConfig) -> None:
        self.config = config.normalized()

    def bearing_from_detection(
        self,
        detection: DetectionResult,
        *,
        timestamp_s: float,
    ) -> BearingObservation | None:
        if detection.target_center is None:
            return None

        fx, fy, cx, cy = self._resolved_intrinsics()
        pixel_x, pixel_y = detection.target_center
        normalized_x = (pixel_x - cx) / fx
        normalized_y = (pixel_y - cy) / fy

        direction = np.array(
            [normalized_x, -normalized_y, 1.0],
            dtype=np.float64,
        )
        direction /= np.linalg.norm(direction)
        azimuth = math.atan2(direction[0], direction[2])
        elevation = math.atan2(direction[1], direction[2])

        return BearingObservation(
            pixel_x=float(pixel_x),
            pixel_y=float(pixel_y),
            azimuth_rad=azimuth,
            elevation_rad=elevation,
            direction_arm=direction,
            confidence=detection.confidence,
            timestamp_s=timestamp_s,
        )

    def _resolved_intrinsics(self) -> tuple[float, float, float, float]:
        cx = self.config.cx if self.config.cx is not None else self.config.frame_width / 2.0
        cy = self.config.cy if self.config.cy is not None else self.config.frame_height / 2.0
        fx = self.config.fx
        fy = self.config.fy

        if fx is None:
            fx = self.config.frame_width / (
                2.0 * math.tan(math.radians(self.config.horizontal_fov_deg) / 2.0)
            )
        if fy is None:
            vertical_fov = self.config.vertical_fov_deg
            if vertical_fov is None:
                aspect_ratio = self.config.frame_height / float(self.config.frame_width)
                vertical_fov = math.degrees(
                    2.0
                    * math.atan(
                        math.tan(math.radians(self.config.horizontal_fov_deg) / 2.0)
                        * aspect_ratio
                    )
                )
            fy = self.config.frame_height / (
                2.0 * math.tan(math.radians(vertical_fov) / 2.0)
            )
        return float(fx), float(fy), float(cx), float(cy)


class PositionFusionEngine:
    """Fuse UWB ranges with the camera bearing into an arm-frame position."""

    def __init__(self, config: TrackerConfig) -> None:
        self.config = config.normalized()
        self.anchor_positions = {
            anchor.anchor_id: np.array(anchor.position_m, dtype=np.float64)
            for anchor in self.config.anchors
        }
        self.camera_position = np.array(self.config.camera.position_m, dtype=np.float64)
        self.gripper_position = np.array(self.config.gripper_position_m, dtype=np.float64)
        self.last_position: np.ndarray | None = None

    def estimate(
        self,
        measurements: Sequence[AnchorMeasurement],
        bearing: BearingObservation | None,
    ) -> PositionEstimate | None:
        latest_measurements = self._latest_by_anchor(measurements)
        range_inputs = [
            (
                anchor_id,
                self.anchor_positions[anchor_id],
                latest_measurements[anchor_id].distance_m,
            )
            for anchor_id in latest_measurements
            if anchor_id in self.anchor_positions
        ]

        if not range_inputs:
            return None

        initial_guess = self._initial_guess(range_inputs, bearing)
        if initial_guess is None:
            return None

        position, residual = self._solve_position(range_inputs, bearing, initial_guess)

        if self.last_position is not None and 0.0 < self.config.smoothing_alpha < 1.0:
            alpha = self.config.smoothing_alpha
            position = (alpha * position) + ((1.0 - alpha) * self.last_position)

        self.last_position = position
        distance_to_gripper = float(np.linalg.norm(position - self.gripper_position))
        return PositionEstimate(
            position_m=(float(position[0]), float(position[1]), float(position[2])),
            method=self._method_name(len(range_inputs), bearing is not None),
            residual=float(residual),
            range_count=len(range_inputs),
            used_camera=bearing is not None,
            distance_to_gripper_m=distance_to_gripper,
            capture_ready=distance_to_gripper <= self.config.capture_radius_m,
        )

    def _initial_guess(
        self,
        range_inputs: Sequence[tuple[str, np.ndarray, float]],
        bearing: BearingObservation | None,
    ) -> np.ndarray | None:
        if self.last_position is not None:
            return self.last_position.copy()
        if bearing is not None:
            target_distance = float(np.mean([distance for _, _, distance in range_inputs]))
            return self.camera_position + (bearing.direction_arm * target_distance)
        if len(range_inputs) >= 3:
            anchor_stack = np.array([anchor_position for _, anchor_position, _ in range_inputs])
            return np.mean(anchor_stack, axis=0)
        return None

    def _solve_position(
        self,
        range_inputs: Sequence[tuple[str, np.ndarray, float]],
        bearing: BearingObservation | None,
        initial_guess: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        point = initial_guess.astype(np.float64)
        direction = None if bearing is None else bearing.direction_arm

        for _ in range(15):
            residuals: list[float] = []
            jacobian_rows: list[np.ndarray] = []

            for _, anchor_position, distance_m in range_inputs:
                delta = point - anchor_position
                norm = float(np.linalg.norm(delta))
                if norm < 1e-9:
                    norm = 1e-9
                residuals.append((norm - distance_m) * self.config.range_weight)
                jacobian_rows.append((delta / norm) * self.config.range_weight)

            if direction is not None:
                projection = np.eye(3, dtype=np.float64) - np.outer(direction, direction)
                camera_residual = projection @ (point - self.camera_position)
                for row_index in range(3):
                    residuals.append(camera_residual[row_index] * self.config.vision_weight)
                    jacobian_rows.append(projection[row_index] * self.config.vision_weight)

            residual_vec = np.array(residuals, dtype=np.float64)
            jacobian = np.vstack(jacobian_rows)
            lhs = jacobian.T @ jacobian + (1e-4 * np.eye(3, dtype=np.float64))
            rhs = jacobian.T @ residual_vec
            step = np.linalg.solve(lhs, rhs)
            point = point - step

            if direction is not None:
                camera_depth = float(np.dot(point - self.camera_position, direction))
                if camera_depth < 0.0:
                    point = self.camera_position + (direction * 0.01)

            if float(np.linalg.norm(step)) < 1e-4:
                break

        residual = self._rms_residual(point, range_inputs, bearing)
        return point, residual

    def _rms_residual(
        self,
        point: np.ndarray,
        range_inputs: Sequence[tuple[str, np.ndarray, float]],
        bearing: BearingObservation | None,
    ) -> float:
        residuals: list[float] = []
        for _, anchor_position, distance_m in range_inputs:
            residuals.append(abs(float(np.linalg.norm(point - anchor_position)) - distance_m))
        if bearing is not None:
            projection = np.eye(3, dtype=np.float64) - np.outer(
                bearing.direction_arm,
                bearing.direction_arm,
            )
            residuals.append(float(np.linalg.norm(projection @ (point - self.camera_position))))
        if not residuals:
            return 0.0
        return float(np.sqrt(np.mean(np.square(residuals))))

    def _latest_by_anchor(
        self,
        measurements: Iterable[AnchorMeasurement],
    ) -> dict[str, AnchorMeasurement]:
        latest: dict[str, AnchorMeasurement] = {}
        for measurement in measurements:
            current = latest.get(measurement.anchor_id)
            if current is None or measurement.timestamp_s >= current.timestamp_s:
                latest[measurement.anchor_id] = measurement
        return latest

    def _method_name(self, range_count: int, used_camera: bool) -> str:
        if used_camera and range_count == 1:
            return "ray_sphere_fusion"
        if used_camera:
            return "multilateration_plus_bearing"
        return "multilateration_only"


class DroneTrackingSystem:
    def __init__(self, config: TrackerConfig) -> None:
        self.config = config.normalized()
        self.detector = BlackColorModel(self.config.detector)
        self.camera_model = CameraModel(self.config.camera)
        self.fusion = PositionFusionEngine(self.config)
        self._latest_measurements: dict[str, AnchorMeasurement] = {}

    def process(
        self,
        frame: np.ndarray,
        *,
        measurements: Sequence[AnchorMeasurement],
        timestamp_s: float,
    ) -> TrackingFrameResult:
        detection = self.detector.detect(frame)
        bearing = self.camera_model.bearing_from_detection(detection, timestamp_s=timestamp_s)

        for measurement in measurements:
            self._latest_measurements[measurement.anchor_id] = measurement

        estimate = self.fusion.estimate(list(self._latest_measurements.values()), bearing)
        return TrackingFrameResult(
            detection=detection,
            bearing=bearing,
            estimate=estimate,
            measurements=list(self._latest_measurements.values()),
        )

    def annotate(self, frame: np.ndarray, result: TrackingFrameResult) -> np.ndarray:
        annotated = self.detector.annotate(frame, result.detection)
        line_y = 160
        if result.bearing is not None:
            cv2.putText(
                annotated,
                (
                    f"bearing az={math.degrees(result.bearing.azimuth_rad):.1f}deg "
                    f"el={math.degrees(result.bearing.elevation_rad):.1f}deg"
                ),
                (20, line_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            line_y += 30
        if result.estimate is not None:
            x, y, z = result.estimate.position_m
            cv2.putText(
                annotated,
                f"pos(m)=({x:.2f}, {y:.2f}, {z:.2f}) via {result.estimate.method}",
                (20, line_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            line_y += 30
            cv2.putText(
                annotated,
                (
                    f"dist_to_gripper={result.estimate.distance_to_gripper_m:.2f}m "
                    f"capture_ready={result.estimate.capture_ready}"
                ),
                (20, line_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 220, 0) if result.estimate.capture_ready else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return annotated
