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


def test_position_fusion_estimates_target_from_single_range_and_bearing() -> None:
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

    estimate = engine.estimate([measurement], bearing)
    assert estimate is not None
    assert math.isclose(estimate.position_m[0], 0.0, abs_tol=0.05)
    assert math.isclose(estimate.position_m[1], 0.0, abs_tol=0.05)
    assert math.isclose(estimate.position_m[2], 1.0, abs_tol=0.05)
    assert math.isclose(estimate.distance_to_gripper_m, 0.0, abs_tol=0.05)
    assert estimate.capture_ready is True
