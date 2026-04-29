import numpy as np

from black_vision.config import DetectorConfig
from black_vision.detector import BlackColorModel


def test_black_patch_is_detected() -> None:
    frame = np.full((120, 120, 3), 255, dtype=np.uint8)
    frame[30:90, 30:90] = 0

    model = BlackColorModel(
        DetectorConfig(
            value_max=40,
            min_area=200,
            blur_kernel=1,
            morph_kernel=1,
            black_ratio_threshold=0.01,
        )
    )
    result = model.detect(frame)

    assert result.detected is True
    assert result.black_ratio > 0.20
    assert len(result.boxes) == 1
    assert result.target_center == (60.0, 60.0)
    assert result.target_box == (30, 30, 60, 60)
    assert result.confidence > 0.0


def test_bright_frame_is_not_detected() -> None:
    frame = np.full((120, 120, 3), 240, dtype=np.uint8)

    model = BlackColorModel(
        DetectorConfig(
            value_max=40,
            min_area=200,
            blur_kernel=1,
            morph_kernel=1,
            black_ratio_threshold=0.01,
        )
    )
    result = model.detect(frame)

    assert result.detected is False
    assert result.black_ratio == 0.0
    assert result.boxes == []
    assert result.target_center is None
    assert result.confidence == 0.0
