# Black Vision Tracker

Standalone Python project for detecting the color black from a camera feed, including ESP32-CAM streams and fused Reyax UWB tracking for a drone-grabber system.

The detector uses a lightweight computer-vision model rather than a trained neural network:

- convert each frame to HSV
- threshold low-value pixels that are dark enough to count as black
- clean up the mask with morphology
- highlight large black regions with bounding boxes

This approach is usually the right fit for "detect black" because lighting and exposure matter more than model size. The project includes a live calibration mode so you can tune thresholds for your room and camera.

## Features

- live webcam preview
- ESP32-CAM stream support through `--camera-source`
- real-time black-region detection
- bounding boxes around black objects or areas
- mask preview for debugging thresholds
- calibration UI with trackbars
- JSON config save/load
- Reyax `RYUW122_Lite` UART polling for distance measurements
- UWB-first tracking that prefers 3-anchor trilateration before falling back to vision
- fused camera-bearing + UWB ranging for 3D arm-frame tracking when only partial UWB is available
- configurable grab radius for drone capture

## Project Layout

```text
vision/black-vision-tracker/
  black_vision/
    cli.py
    config.py
    detector.py
    tracker_config.py
    tracking.py
    uwb.py
  tests/
    test_detector.py
    test_tracking.py
    test_uwb.py
```

## Setup

```bash
cd /Users/spershgoyal/Documents/Playground/drone-catch-system/vision/black-vision-tracker
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## Quick Start

Run live detection with the default settings:

```bash
black-vision detect
```

If your camera is an ESP32-CAM, use its stream URL:

```bash
black-vision detect --camera-source http://192.168.4.1:81/stream
```

Open calibration mode so you can tune the detector for your webcam and lighting:

```bash
black-vision calibrate --output ./camera_config.json
```

Then reuse the saved config for normal detection:

```bash
black-vision detect --config ./camera_config.json
```

For fused arm + drone tracking, start from the included example config:

```bash
cp ./tracker_config.example.json ./tracker_config.json
black-vision track --tracker-config ./tracker_config.json --print-json
```

The default example is arranged as a UWB-first layout with three enabled fixed anchors, an optional disabled tip anchor placeholder, numeric arm-v4-style addresses, and `anchor_send_mode` set to `address_only`.

## Commands

### Detect

```bash
black-vision detect --camera-index 0
```

Or, for an ESP32-CAM stream:

```bash
black-vision detect --camera-source http://192.168.4.1:81/stream
```

Controls:

- `q`: quit

### Calibrate

```bash
black-vision calibrate --camera-index 0 --output ./camera_config.json
```

Or:

```bash
black-vision calibrate --camera-source http://192.168.4.1:81/stream --output ./camera_config.json
```

Controls:

- `s`: save the current thresholds to the output path
- `q`: quit

### Track

```bash
black-vision track --tracker-config ./tracker_config.json
```

Controls:

- `q`: quit

## Notes for ESP32-CAM

- Use the ESP32 only as the camera and stream source. The OpenCV + UWB fusion code should run on the arm-side computer or SBC.
- Most ESP32-CAM examples expose MJPEG at a URL like `http://<ip>:81/stream`.
- Start at `640x480` and tune thresholds there before pushing resolution higher.
- Lock exposure and white balance in your ESP32 camera firmware if possible. Black detection becomes much more stable.

## Notes for Reyax RYUW122_Lite

- The tracker accepts both older long-form replies like `+ANCHOR_RCV=DRONE001,4,PING,183 cm,-78 dBm` and the shorter arm-v4 format `+ANCHOR_RCV=99,183,-78`.
- The tracker can poll tags with either the older payload form `AT+ANCHOR_SEND=<tag>,<len>,<payload>` or the arm-v4 short form `AT+ANCHOR_SEND=<tag>`.
- The anchor must actively poll the drone tag with `AT+ANCHOR_SEND=...`.
- Fresh UWB anchors are preferred over vision. If at least three non-tip anchors are fresh, the tracker does pure trilateration first.
- If UWB drops below the configured freshness threshold, the tracker falls back to a camera-only estimate using the last known depth or `vision_fallback_distance_m`.
- If you use multiple anchors, stagger their polling so UWB transmissions do not overlap.

## Config Fields

The saved JSON config includes:

- `camera_index`
- `camera_source`
- `frame_width`
- `frame_height`
- `value_max`
- `saturation_max`
- `min_area`
- `morph_kernel`
- `blur_kernel`
- `black_ratio_threshold`
- `show_mask`
- `draw_boxes`

The tracker JSON adds:

- camera intrinsics or horizontal field of view
- anchor UART ports, arm-frame positions, and anchor roles such as `base` or `tip`
- anchor send mode for long-form or short-form Reyax polling
- drone tag address
- Reyax network / channel / bandwidth settings
- gripper position and capture radius
- UWB freshness, residual rejection, and fallback-distance settings

## Run Tests

```bash
pytest
```
