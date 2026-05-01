import math

import numpy as np

from black_vision.config import DetectorConfig
from black_vision.detector import BlackColorModel
from black_vision.tracker_config import AnchorConfig, CameraConfig, DroneTagConfig, TrackerConfig
from black_vision.tracking import CameraModel, PositionFusionEngine
from black_vision.uwb import AnchorMeasurement


def test_camera_model_converts_black_target_into_centered_bearing() -> None:
    frame = np.full((120, 120, 3), 255, dtype=np.uint8)
    frame[40:80, 40:80] = 0

    model = BlackColorModel(
        DetectorConfig(
            frame_width=120,
            frame_height=120,
            value_max=40,
            min_area=100,
            blur_kernel=1,
            morph_kernel=1,
            black_ratio_threshold=0.01,
        )
    )
    detection = model.detect(frame)
    camera_model = CameraModel(
        CameraConfig(
            frame_width=120,
            frame_height=120,
            horizontal_fov_deg=75.0,
        )
    )

    bearing = camera_model.bearing_from_detection(detection, timestamp_s=1.0)
    assert bearing is not None
    assert abs(bearing.azimuth_rad) < 1e-6
    assert abs(bearing.elevation_rad) < 1e-6


def test_position_fusion_prefers_uwb_trilateration_with_three_fresh_anchors() -> None:
    config = TrackerConfig(
        detector=DetectorConfig(frame_width=100, frame_height=100),
        camera=CameraConfig(
            frame_width=100,
            frame_height=100,
            horizontal_fov_deg=75.0,
            position_m=(0.0, 0.0, 0.0),
        ),
        anchors=[
            AnchorConfig(
                anchor_id="a0",
                address="REYAX001",
                serial_port="/dev/null",
                position_m=(0.0, 0.0, 0.0),
            ),
            AnchorConfig(
                anchor_id="a1",
                address="REYAX002",
                serial_port="/dev/null",
                position_m=(1.0, 0.0, 0.0),
            ),
            AnchorConfig(
                anchor_id="a2",
                address="REYAX003",
                serial_port="/dev/null",
                position_m=(0.0, 1.0, 0.0),
            ),
        ],
        drone_tag=DroneTagConfig(address="DRONE001"),
        gripper_position_m=(0.0, 0.0, 0.25),
        capture_radius_m=0.1,
        smoothing_alpha=1.0,
        minimum_uwb_anchors=3,
        max_uwb_residual_m=0.1,
    ).normalized()

    target = np.array([0.30, 0.40, 0.80], dtype=np.float64)
    engine = PositionFusionEngine(config)
    measurements = [
        _measurement("a0", tuple(config.anchors[0].position_m), target, timestamp_s=1.0),
        _measurement("a1", tuple(config.anchors[1].position_m), target, timestamp_s=1.0),
        _measurement("a2", tuple(config.anchors[2].position_m), target, timestamp_s=1.0),
    ]

    estimate = engine.estimate(measurements, None, timestamp_s=1.0)

    assert estimate is not None
    assert estimate.source == "uwb"
    assert estimate.method == "uwb_trilateration"
    assert estimate.used_camera is False
    assert estimate.range_count == 3
    assert math.isclose(estimate.position_m[0], target[0], abs_tol=0.03)
    assert math.isclose(estimate.position_m[1], target[1], abs_tol=0.03)
    assert math.isclose(estimate.position_m[2], target[2], abs_tol=0.03)


def test_position_fusion_uses_hybrid_when_only_one_anchor_is_available() -> None:
    frame = np.full((100, 100, 3), 255, dtype=np.uint8)
    frame[40:60, 40:60] = 0
    detection = BlackColorModel(
        DetectorConfig(
            frame_width=100,
            frame_height=100,
            value_max=40,
            min_area=10,
            blur_kernel=1,
            morph_kernel=1,
            black_ratio_threshold=0.01,
        )
    ).detect(frame)

    config = TrackerConfig(
        detector=DetectorConfig(frame_width=100, frame_height=100),
        camera=CameraConfig(
            frame_width=100,
            frame_height=100,
            horizontal_fov_deg=75.0,
            position_m=(0.0, 0.0, 0.0),
        ),
        anchors=[
            AnchorConfig(
                anchor_id="arm_anchor",
                address="REYAX001",
                serial_port="/dev/null",
                position_m=(0.0, 0.0, 0.0),
            )
        ],
        drone_tag=DroneTagConfig(address="DRONE001"),
        gripper_position_m=(0.0, 0.0, 1.0),
        capture_radius_m=0.2,
        smoothing_alpha=1.0,
        max_hybrid_residual_m=0.1,
    ).normalized()

    engine = PositionFusionEngine(config)
    measurement = AnchorMeasurement(
        anchor_id="arm_anchor",
        anchor_address="REYAX001",
        tag_address="DRONE001",
        payload="PING",
        payload_length=4,
        distance_m=1.0,
        rssi=None,
        timestamp_s=1.0,
        raw_line="+ANCHOR_RCV=DRONE001,4,PING,100 cm",
    )
    bearing = CameraModel(config.camera).bearing_from_detection(detection, timestamp_s=1.0)

    assert bearing is not None

    estimate = engine.estimate([measurement], bearing, timestamp_s=1.0)
    assert estimate is not None
    assert estimate.source == "hybrid"
    assert estimate.method == "uwb_plus_vision"
    assert math.isclose(estimate.position_m[0], 0.0, abs_tol=0.05)
    assert math.isclose(estimate.position_m[1], 0.0, abs_tol=0.05)
    assert math.isclose(estimate.position_m[2], 1.0, abs_tol=0.05)
    assert math.isclose(estimate.distance_to_gripper_m, 0.0, abs_tol=0.05)
    assert estimate.capture_ready is True


def test_position_fusion_falls_back_to_vision_when_uwb_is_stale() -> None:
    frame = np.full((100, 100, 3), 255, dtype=np.uint8)
    frame[40:60, 40:60] = 0
    detection = BlackColorModel(
        DetectorConfig(
            frame_width=100,
            frame_height=100,
            value_max=40,
            min_area=10,
            blur_kernel=1,
            morph_kernel=1,
            black_ratio_threshold=0.01,
        )
    ).detect(frame)

    config = TrackerConfig(
        detector=DetectorConfig(frame_width=100, frame_height=100),
        camera=CameraConfig(
            frame_width=100,
            frame_height=100,
            horizontal_fov_deg=75.0,
            position_m=(0.0, 0.0, 0.0),
        ),
        anchors=[
            AnchorConfig(
                anchor_id="arm_anchor",
                address="REYAX001",
                serial_port="/dev/null",
                position_m=(0.0, 0.0, 0.0),
            )
        ],
        drone_tag=DroneTagConfig(address="DRONE001"),
        gripper_position_m=(0.0, 0.0, 1.8),
        capture_radius_m=0.2,
        smoothing_alpha=1.0,
        uwb_stale_after_s=0.1,
        vision_fallback_distance_m=1.8,
    ).normalized()

    engine = PositionFusionEngine(config)
    measurement = AnchorMeasurement(
        anchor_id="arm_anchor",
        anchor_address="REYAX001",
        tag_address="DRONE001",
        payload="PING",
        payload_length=4,
        distance_m=0.4,
        rssi=None,
        timestamp_s=1.0,
        raw_line="+ANCHOR_RCV=DRONE001,4,PING,40 cm",
    )
    bearing = CameraModel(config.camera).bearing_from_detection(detection, timestamp_s=2.0)

    assert bearing is not None

    estimate = engine.estimate([measurement], bearing, timestamp_s=2.0)
    assert estimate is not None
    assert estimate.source == "vision"
    assert estimate.method == "vision_fallback"
    assert estimate.range_count == 0
    assert math.isclose(estimate.position_m[0], 0.0, abs_tol=0.05)
    assert math.isclose(estimate.position_m[1], 0.0, abs_tol=0.05)
    assert math.isclose(estimate.position_m[2], 1.8, abs_tol=0.05)


def _measurement(
    anchor_id: str,
    anchor_position: tuple[float, float, float],
    target_position: np.ndarray,
    *,
    timestamp_s: float,
) -> AnchorMeasurement:
    anchor_vec = np.array(anchor_position, dtype=np.float64)
    distance_m = float(np.linalg.norm(target_position - anchor_vec))
    return AnchorMeasurement(
        anchor_id=anchor_id,
        anchor_address=f"{anchor_id.upper():0<8}"[:8],
        tag_address="DRONE001",
        payload="PING",
        payload_length=4,
        distance_m=distance_m,
        rssi=-70.0,
        timestamp_s=timestamp_s,
        raw_line="+ANCHOR_RCV=DRONE001,4,PING,100 cm,-70 dBm",
    )
