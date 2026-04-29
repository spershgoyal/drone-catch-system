from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(slots=True)
class DetectorConfig:
    camera_index: int = 0
    camera_source: str | None = None
    frame_width: int = 1280
    frame_height: int = 720
    value_max: int = 60
    saturation_max: int = 255
    min_area: int = 1200
    morph_kernel: int = 5
    blur_kernel: int = 5
    black_ratio_threshold: float = 0.01
    show_mask: bool = True
    draw_boxes: bool = True

    def normalized(self) -> "DetectorConfig":
        return DetectorConfig(
            camera_index=max(0, int(self.camera_index)),
            camera_source=_normalize_camera_source(self.camera_source),
            frame_width=max(1, int(self.frame_width)),
            frame_height=max(1, int(self.frame_height)),
            value_max=_clamp(int(self.value_max), 0, 255),
            saturation_max=_clamp(int(self.saturation_max), 0, 255),
            min_area=max(1, int(self.min_area)),
            morph_kernel=_odd_at_least_one(int(self.morph_kernel)),
            blur_kernel=_odd_at_least_one(int(self.blur_kernel)),
            black_ratio_threshold=max(0.0, min(float(self.black_ratio_threshold), 1.0)),
            show_mask=bool(self.show_mask),
            draw_boxes=bool(self.draw_boxes),
        )

    def to_dict(self) -> dict[str, object]:
        return asdict(self.normalized())


def load_config(path: str | Path) -> DetectorConfig:
    config_path = Path(path)
    payload = json.loads(config_path.read_text())
    return DetectorConfig(**payload).normalized()


def save_config(config: DetectorConfig, path: str | Path) -> Path:
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(json.dumps(config.to_dict(), indent=2) + "\n")
    return config_path


def _clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(value, upper))


def _odd_at_least_one(value: int) -> int:
    value = max(1, value)
    if value % 2 == 0:
        value += 1
    return value


def _normalize_camera_source(value: str | None) -> str | None:
    if value is None:
        return None
    source = value.strip()
    return source or None
