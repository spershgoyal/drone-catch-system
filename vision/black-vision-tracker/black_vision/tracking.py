from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations
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
    source: str
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
            payload["source"] = self.estimate.source
            payload["method"] = self.estimate.method
            payload["residual"] = round(self.estimate.residual, 6)
            payload["range_count"] = self.estimate.range_count
        return payload


@dataclass(slots=True)
class RangeInput:
    anchor_id: str
    anchor_position: np.ndarray
    distance_m: float
    role: str


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
    """Prefer fresh UWB trilateration, then hybrid UWB+vision, then vision fallback."""

    def __init__(self, config: TrackerConfig) -> None:
        self.config = config.normalized()
        self.anchor_configs = {
            anchor.anchor_id: anchor
            for anchor in self.config.anchors
        }
        self.anchor_positions = {
            anchor.anchor_id: np.array(anchor.position_m, dtype=np.float64)
            for anchor in self.config.anchors
        }
        self.camera_position = np.array(self.config.camera.position_m, dtype=np.float64)
        self.gripper_position = np.array(self.config.gripper_position_m, dtype=np.float64)
        self.last_position: np.ndarray | None = None

    def current_measurements(
        self,
        measurements: Iterable[AnchorMeasurement],
        *,
        timestamp_s: float,
    ) -> list[AnchorMeasurement]:
        latest = self._latest_by_anchor(measurements)
        fresh = [
            measurement
            for measurement in latest.values()
            if measurement.anchor_id in self.anchor_configs
            and (timestamp_s - measurement.timestamp_s) <= self.config.uwb_stale_after_s
        ]
        fresh.sort(key=lambda measurement: measurement.anchor_id)
        return fresh

    def estimate(
        self,
        measurements: Sequence[AnchorMeasurement],
        bearing: BearingObservation | None,
        *,
        timestamp_s: float,
    ) -> PositionEstimate | None:
        fresh_measurements = self.current_measurements(measurements, timestamp_s=timestamp_s)
        range_inputs = self._range_inputs(fresh_measurements)

        if self.config.prefer_uwb:
            estimate = self._estimate_uwb_trilateration(range_inputs)
            if estimate is not None:
                return estimate
            estimate = self._estimate_hybrid(range_inputs, bearing)
            if estimate is not None:
                return estimate
            return self._estimate_vision_only(bearing)

        estimate = self._estimate_hybrid(range_inputs, bearing)
        if estimate is not None:
            return estimate
        estimate = self._estimate_uwb_trilateration(range_inputs)
        if estimate is not None:
            return estimate
        return self._estimate_vision_only(bearing)

    def _estimate_uwb_trilateration(
        self,
        range_inputs: Sequence[RangeInput],
    ) -> PositionEstimate | None:
        positional_inputs = [range_input for range_input in range_inputs if range_input.role != "tip"]
        if len(positional_inputs) < self.config.minimum_uwb_anchors:
            return None

        initial_guess = self._initial_guess_from_ranges(positional_inputs)
        if initial_guess is None:
            return None

        position, residual = self._solve_position(positional_inputs, None, initial_guess)
        if residual > self.config.max_uwb_residual_m:
            return None
        return self._build_estimate(
            position=position,
            source="uwb",
            method="uwb_trilateration",
            residual=residual,
            range_count=len(positional_inputs),
            used_camera=False,
        )

    def _estimate_hybrid(
        self,
        range_inputs: Sequence[RangeInput],
        bearing: BearingObservation | None,
    ) -> PositionEstimate | None:
        if bearing is None or not range_inputs:
            return None

        initial_guess = self._initial_guess(range_inputs, bearing)
        if initial_guess is None:
            return None

        position, residual = self._solve_position(range_inputs, bearing, initial_guess)
        if residual > self.config.max_hybrid_residual_m:
            return None
        return self._build_estimate(
            position=position,
            source="hybrid",
            method="uwb_plus_vision",
            residual=residual,
            range_count=len(range_inputs),
            used_camera=True,
        )

    def _estimate_vision_only(
        self,
        bearing: BearingObservation | None,
    ) -> PositionEstimate | None:
        if bearing is None:
            return None

        distance_m = self._vision_fallback_distance()
        position = self.camera_position + (bearing.direction_arm * distance_m)
        residual = 0.0
        return self._build_estimate(
            position=position,
            source="vision",
            method="vision_fallback",
            residual=residual,
            range_count=0,
            used_camera=True,
        )

    def _initial_guess(
        self,
        range_inputs: Sequence[RangeInput],
        bearing: BearingObservation | None,
    ) -> np.ndarray | None:
        if self.last_position is not None:
            return self.last_position.copy()
        if bearing is not None:
            target_distance = float(np.mean([range_input.distance_m for range_input in range_inputs]))
            return self.camera_position + (bearing.direction_arm * target_distance)
        return self._initial_guess_from_ranges(range_inputs)

    def _initial_guess_from_ranges(
        self,
        range_inputs: Sequence[RangeInput],
    ) -> np.ndarray | None:
        if self.last_position is not None:
            return self.last_position.copy()

        trilat_candidate = self._seed_from_three_anchors(range_inputs)
        if trilat_candidate is not None:
            return trilat_candidate

        if len(range_inputs) >= 3:
            anchor_stack = np.array(
                [range_input.anchor_position for range_input in range_inputs],
                dtype=np.float64,
            )
            return np.mean(anchor_stack, axis=0)
        return None

    def _seed_from_three_anchors(
        self,
        range_inputs: Sequence[RangeInput],
    ) -> np.ndarray | None:
        positional_inputs = [range_input for range_input in range_inputs if range_input.role != "tip"]
        if len(positional_inputs) < 3:
            return None

        for trio in combinations(positional_inputs, 3):
            candidate = self._trilaterate_three_anchors(trio, positional_inputs)
            if candidate is not None:
                return candidate
        return None

    def _trilaterate_three_anchors(
        self,
        trio: Sequence[RangeInput],
        all_inputs: Sequence[RangeInput],
    ) -> np.ndarray | None:
        anchor_a, anchor_b, anchor_c = trio
        point_a = anchor_a.anchor_position
        point_b = anchor_b.anchor_position
        point_c = anchor_c.anchor_position
        ex = point_b - point_a
        distance_ab = float(np.linalg.norm(ex))
        if distance_ab < 1e-9:
            return None
        ex /= distance_ab

        point_ac = point_c - point_a
        projection_i = float(np.dot(ex, point_ac))
        ey = point_ac - (projection_i * ex)
        ey_norm = float(np.linalg.norm(ey))
        if ey_norm < 1e-9:
            return None
        ey /= ey_norm
        ez = np.cross(ex, ey)
        projection_j = float(np.dot(ey, point_ac))
        if abs(projection_j) < 1e-9:
            return None

        distance_a = anchor_a.distance_m
        distance_b = anchor_b.distance_m
        distance_c = anchor_c.distance_m
        solved_x = (
            (distance_a * distance_a) - (distance_b * distance_b) + (distance_ab * distance_ab)
        ) / (2.0 * distance_ab)
        solved_y = (
            (
                (distance_a * distance_a)
                - (distance_c * distance_c)
                + (projection_i * projection_i)
                + (projection_j * projection_j)
            )
            / (2.0 * projection_j)
        ) - ((projection_i / projection_j) * solved_x)
        solved_z_sq = (distance_a * distance_a) - (solved_x * solved_x) - (solved_y * solved_y)
        if solved_z_sq < -1e-6:
            return None
        solved_z = math.sqrt(max(0.0, solved_z_sq))

        candidate_one = point_a + (solved_x * ex) + (solved_y * ey) + (solved_z * ez)
        candidate_two = point_a + (solved_x * ex) + (solved_y * ey) - (solved_z * ez)
        return self._pick_best_candidate(
            (candidate_one, candidate_two),
            all_inputs,
        )

    def _pick_best_candidate(
        self,
        candidates: Sequence[np.ndarray],
        range_inputs: Sequence[RangeInput],
    ) -> np.ndarray | None:
        best_candidate: np.ndarray | None = None
        best_score: tuple[float, float] | None = None
        for candidate in candidates:
            residual = self._rms_residual(candidate, range_inputs, None)
            score = (residual, -float(candidate[2]))
            if best_score is None or score < best_score:
                best_score = score
                best_candidate = candidate
        return None if best_candidate is None else best_candidate.copy()

    def _solve_position(
        self,
        range_inputs: Sequence[RangeInput],
        bearing: BearingObservation | None,
        initial_guess: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        point = initial_guess.astype(np.float64)
        direction = None if bearing is None else bearing.direction_arm

        for _ in range(15):
            residuals: list[float] = []
            jacobian_rows: list[np.ndarray] = []

            for range_input in range_inputs:
                delta = point - range_input.anchor_position
                norm = float(np.linalg.norm(delta))
                if norm < 1e-9:
                    norm = 1e-9
                residuals.append((norm - range_input.distance_m) * self.config.range_weight)
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
        range_inputs: Sequence[RangeInput],
        bearing: BearingObservation | None,
    ) -> float:
        residuals: list[float] = []
        for range_input in range_inputs:
            residuals.append(
                abs(float(np.linalg.norm(point - range_input.anchor_position)) - range_input.distance_m)
            )
        if bearing is not None:
            projection = np.eye(3, dtype=np.float64) - np.outer(
                bearing.direction_arm,
                bearing.direction_arm,
            )
            residuals.append(float(np.linalg.norm(projection @ (point - self.camera_position))))
        if not residuals:
            return 0.0
        return float(np.sqrt(np.mean(np.square(residuals))))

    def _range_inputs(
        self,
        measurements: Sequence[AnchorMeasurement],
    ) -> list[RangeInput]:
        inputs: list[RangeInput] = []
        for measurement in measurements:
            anchor_config = self.anchor_configs.get(measurement.anchor_id)
            if anchor_config is None:
                continue
            inputs.append(
                RangeInput(
                    anchor_id=measurement.anchor_id,
                    anchor_position=self.anchor_positions[measurement.anchor_id],
                    distance_m=measurement.distance_m,
                    role=anchor_config.role,
                )
            )
        return inputs

    def _vision_fallback_distance(self) -> float:
        if self.last_position is not None:
            last_distance = float(np.linalg.norm(self.last_position - self.camera_position))
            return max(0.05, last_distance)
        return self.config.vision_fallback_distance_m

    def _build_estimate(
        self,
        *,
        position: np.ndarray,
        source: str,
        method: str,
        residual: float,
        range_count: int,
        used_camera: bool,
    ) -> PositionEstimate:
        if self.last_position is not None and 0.0 < self.config.smoothing_alpha < 1.0:
            alpha = self.config.smoothing_alpha
            position = (alpha * position) + ((1.0 - alpha) * self.last_position)

        self.last_position = position.copy()
        distance_to_gripper = float(np.linalg.norm(position - self.gripper_position))
        return PositionEstimate(
            position_m=(float(position[0]), float(position[1]), float(position[2])),
            source=source,
            method=method,
            residual=float(residual),
            range_count=range_count,
            used_camera=used_camera,
            distance_to_gripper_m=distance_to_gripper,
            capture_ready=distance_to_gripper <= self.config.capture_radius_m,
        )

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

        current_measurements = self.fusion.current_measurements(
            self._latest_measurements.values(),
            timestamp_s=timestamp_s,
        )
        estimate = self.fusion.estimate(
            current_measurements,
            bearing,
            timestamp_s=timestamp_s,
        )
        return TrackingFrameResult(
            detection=detection,
            bearing=bearing,
            estimate=estimate,
            measurements=current_measurements,
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
                (
                    f"pos(m)=({x:.2f}, {y:.2f}, {z:.2f}) "
                    f"via {result.estimate.method}"
                ),
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
                    f"source={result.estimate.source} "
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
