from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import DetectorConfig


@dataclass(slots=True)
class DetectionResult:
    detected: bool
    black_ratio: float
    mask: np.ndarray
    boxes: list[tuple[int, int, int, int]]
    contour_areas: list[float]
    target_box: tuple[int, int, int, int] | None
    target_center: tuple[float, float] | None
    target_area: float
    confidence: float


class BlackColorModel:
    """Detect black regions in a BGR frame using HSV thresholding."""

    def __init__(self, config: DetectorConfig) -> None:
        self.config = config.normalized()

    def build_mask(self, frame: np.ndarray) -> np.ndarray:
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("Expected a BGR frame with shape (H, W, 3).")

        working_frame = frame
        if self.config.blur_kernel > 1:
            working_frame = cv2.GaussianBlur(
                working_frame,
                (self.config.blur_kernel, self.config.blur_kernel),
                0,
            )

        hsv_frame = cv2.cvtColor(working_frame, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0], dtype=np.uint8)
        upper = np.array(
            [179, self.config.saturation_max, self.config.value_max],
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv_frame, lower, upper)

        if self.config.morph_kernel > 1:
            kernel = np.ones(
                (self.config.morph_kernel, self.config.morph_kernel),
                dtype=np.uint8,
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def detect(self, frame: np.ndarray) -> DetectionResult:
        mask = self.build_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes: list[tuple[int, int, int, int]] = []
        contour_areas: list[float] = []
        target_box: tuple[int, int, int, int] | None = None
        target_center: tuple[float, float] | None = None
        target_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.min_area:
                continue
            box = cv2.boundingRect(contour)
            boxes.append(box)
            contour_areas.append(area)
            if area > target_area:
                target_area = area
                target_box = box
                x, y, w, h = box
                target_center = (x + (w / 2.0), y + (h / 2.0))

        black_ratio = float(np.count_nonzero(mask)) / float(mask.size)
        detected = bool(boxes) or black_ratio >= self.config.black_ratio_threshold
        area_score = 0.0
        if self.config.min_area > 0:
            area_score = min(1.0, target_area / float(self.config.min_area * 4))
        ratio_score = 0.0
        if self.config.black_ratio_threshold > 0.0:
            ratio_score = min(1.0, black_ratio / self.config.black_ratio_threshold)
        confidence = max(area_score, ratio_score) if detected else 0.0

        return DetectionResult(
            detected=detected,
            black_ratio=black_ratio,
            mask=mask,
            boxes=boxes,
            contour_areas=contour_areas,
            target_box=target_box,
            target_center=target_center,
            target_area=target_area,
            confidence=confidence,
        )

    def annotate(self, frame: np.ndarray, result: DetectionResult) -> np.ndarray:
        annotated = frame.copy()
        status_text = "BLACK DETECTED" if result.detected else "No black target"
        status_color = (0, 200, 0) if result.detected else (0, 0, 220)

        if self.config.draw_boxes:
            for x, y, w, h in result.boxes:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 180, 255), 2)
            if result.target_center is not None:
                center_x = int(round(result.target_center[0]))
                center_y = int(round(result.target_center[1]))
                cv2.drawMarker(
                    annotated,
                    (center_x, center_y),
                    (255, 200, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )

        cv2.putText(
            annotated,
            status_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            status_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"black_ratio={result.black_ratio:.3f}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"value_max={self.config.value_max} min_area={self.config.min_area}",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if result.target_center is not None:
            center_text = (
                f"target=({result.target_center[0]:.1f},{result.target_center[1]:.1f}) "
                f"conf={result.confidence:.2f}"
            )
            cv2.putText(
                annotated,
                center_text,
                (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return annotated
