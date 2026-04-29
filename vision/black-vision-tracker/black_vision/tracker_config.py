from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .config import DetectorConfig


@dataclass(slots=True)
class CameraConfig:
    frame_width: int = 1280
    frame_height: int = 720
    horizontal_fov_deg: float = 75.0
    vertical_fov_deg: float | None = None
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def normalized(
        self,
        *,
        default_width: int | None = None,
        default_height: int | None = None,
    ) -> "CameraConfig":
        frame_width = max(1, int(default_width if default_width is not None else self.frame_width))
        frame_height = max(
            1,
            int(default_height if default_height is not None else self.frame_height),
        )
        horizontal_fov_deg = max(1.0, min(float(self.horizontal_fov_deg), 179.0))
        vertical_fov_deg = (
            None
            if self.vertical_fov_deg is None
            else max(1.0, min(float(self.vertical_fov_deg), 179.0))
        )
        fx = None if self.fx is None else max(1.0, float(self.fx))
        fy = None if self.fy is None else max(1.0, float(self.fy))
        cx = None if self.cx is None else float(self.cx)
        cy = None if self.cy is None else float(self.cy)
        return CameraConfig(
            frame_width=frame_width,
            frame_height=frame_height,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            position_m=_coerce_vector3(self.position_m),
        )


@dataclass(slots=True)
class AnchorConfig:
    anchor_id: str = "arm_main"
    address: str = "REYAX001"
    serial_port: str = "/dev/ttyUSB0"
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    poll_interval_s: float = 0.08
    enabled: bool = True

    def normalized(self) -> "AnchorConfig":
        return AnchorConfig(
            anchor_id=self.anchor_id.strip() or "anchor",
            address=_normalize_ascii_id(self.address, fallback="REYAX001"),
            serial_port=self.serial_port.strip(),
            position_m=_coerce_vector3(self.position_m),
            poll_interval_s=max(0.02, float(self.poll_interval_s)),
            enabled=bool(self.enabled),
        )


@dataclass(slots=True)
class DroneTagConfig:
    address: str = "DRONE001"

    def normalized(self) -> "DroneTagConfig":
        return DroneTagConfig(address=_normalize_ascii_id(self.address, fallback="DRONE001"))


@dataclass(slots=True)
class TrackerConfig:
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    anchors: list[AnchorConfig] = field(default_factory=lambda: [AnchorConfig()])
    drone_tag: DroneTagConfig = field(default_factory=DroneTagConfig)
    gripper_position_m: tuple[float, float, float] = (0.0, 0.0, 0.25)
    capture_radius_m: float = 0.15
    smoothing_alpha: float = 0.35
    vision_weight: float = 2.0
    range_weight: float = 1.0
    serial_baud_rate: int = 115200
    uwb_channel: int = 5
    uwb_bandwidth: int = 1
    uwb_network_id: str = "REYAX123"
    uwb_password: str = "00000000000000000000000000000000"
    anchor_poll_payload: str = "PING"
    anchor_response_timeout_s: float = 0.15
    auto_configure_anchors: bool = True
    enable_rssi: bool = False

    def normalized(self) -> "TrackerConfig":
        detector = self.detector.normalized()
        camera = self.camera.normalized(
            default_width=detector.frame_width,
            default_height=detector.frame_height,
        )
        anchors = [anchor.normalized() for anchor in self.anchors if anchor.enabled]
        if not anchors:
            anchors = [AnchorConfig().normalized()]
        return TrackerConfig(
            detector=detector,
            camera=camera,
            anchors=anchors,
            drone_tag=self.drone_tag.normalized(),
            gripper_position_m=_coerce_vector3(self.gripper_position_m),
            capture_radius_m=max(0.01, float(self.capture_radius_m)),
            smoothing_alpha=max(0.0, min(float(self.smoothing_alpha), 1.0)),
            vision_weight=max(0.1, float(self.vision_weight)),
            range_weight=max(0.1, float(self.range_weight)),
            serial_baud_rate=max(9600, int(self.serial_baud_rate)),
            uwb_channel=5 if int(self.uwb_channel) not in (5, 9) else int(self.uwb_channel),
            uwb_bandwidth=0 if int(self.uwb_bandwidth) == 0 else 1,
            uwb_network_id=_normalize_ascii_id(self.uwb_network_id, fallback="REYAX123"),
            uwb_password=_normalize_password(self.uwb_password),
            anchor_poll_payload=_normalize_payload(self.anchor_poll_payload),
            anchor_response_timeout_s=max(0.05, float(self.anchor_response_timeout_s)),
            auto_configure_anchors=bool(self.auto_configure_anchors),
            enable_rssi=bool(self.enable_rssi),
        )

    def to_dict(self) -> dict[str, object]:
        return asdict(self.normalized())


def load_tracker_config(path: str | Path) -> TrackerConfig:
    config_path = Path(path)
    payload = json.loads(config_path.read_text())
    detector_payload = payload.get("detector", {})
    camera_payload = payload.get("camera", {})
    anchors_payload = payload.get("anchors", [])
    drone_tag_payload = payload.get("drone_tag", {})

    return TrackerConfig(
        detector=DetectorConfig(**detector_payload),
        camera=CameraConfig(**camera_payload),
        anchors=[AnchorConfig(**anchor) for anchor in anchors_payload],
        drone_tag=DroneTagConfig(**drone_tag_payload),
        gripper_position_m=payload.get("gripper_position_m", (0.0, 0.0, 0.25)),
        capture_radius_m=payload.get("capture_radius_m", 0.15),
        smoothing_alpha=payload.get("smoothing_alpha", 0.35),
        vision_weight=payload.get("vision_weight", 2.0),
        range_weight=payload.get("range_weight", 1.0),
        serial_baud_rate=payload.get("serial_baud_rate", 115200),
        uwb_channel=payload.get("uwb_channel", 5),
        uwb_bandwidth=payload.get("uwb_bandwidth", 1),
        uwb_network_id=payload.get("uwb_network_id", "REYAX123"),
        uwb_password=payload.get("uwb_password", "00000000000000000000000000000000"),
        anchor_poll_payload=payload.get("anchor_poll_payload", "PING"),
        anchor_response_timeout_s=payload.get("anchor_response_timeout_s", 0.15),
        auto_configure_anchors=payload.get("auto_configure_anchors", True),
        enable_rssi=payload.get("enable_rssi", False),
    ).normalized()


def save_tracker_config(config: TrackerConfig, path: str | Path) -> Path:
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(json.dumps(config.to_dict(), indent=2) + "\n")
    return config_path


def _coerce_vector3(value: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError("Expected a 3D vector with exactly three values.")
    return (float(value[0]), float(value[1]), float(value[2]))


def _normalize_ascii_id(value: str, *, fallback: str) -> str:
    trimmed = (value or fallback).strip() or fallback
    return trimmed[:8].ljust(8, "0")


def _normalize_password(value: str) -> str:
    trimmed = (value or "").strip().upper()
    if len(trimmed) != 32:
        trimmed = "00000000000000000000000000000000"
    return trimmed


def _normalize_payload(value: str) -> str:
    payload = (value or "PING").encode("ascii", errors="ignore").decode("ascii")
    payload = payload[:12]
    return payload or "PING"
