from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2

from .config import DetectorConfig, load_config, save_config
from .detector import BlackColorModel
from .tracker_config import TrackerConfig, load_tracker_config, save_tracker_config
from .tracking import DroneTrackingSystem
from .uwb import ReyaxSerialAnchor


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="black-vision",
        description="Detect black regions from a camera feed or ESP32 stream.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    detect_parser = subparsers.add_parser("detect", help="Run live black detection.")
    add_shared_arguments(detect_parser)
    detect_parser.set_defaults(func=run_detect)

    calibrate_parser = subparsers.add_parser(
        "calibrate",
        help="Tune thresholds live and optionally save them.",
    )
    add_shared_arguments(calibrate_parser)
    calibrate_parser.add_argument(
        "--output",
        type=Path,
        default=Path("./camera_config.json"),
        help="Where to save the tuned config when you press 's'.",
    )
    calibrate_parser.set_defaults(func=run_calibrate)

    track_parser = subparsers.add_parser(
        "track",
        help="Fuse black-target vision with Reyax UWB ranging for drone tracking.",
    )
    track_parser.add_argument(
        "--tracker-config",
        type=Path,
        default=Path("./tracker_config.json"),
        help="Path to the fused camera + UWB tracker config.",
    )
    track_parser.add_argument(
        "--write-default-tracker-config",
        action="store_true",
        help="Write a starter tracker config to --tracker-config and exit.",
    )
    track_parser.add_argument(
        "--camera-index",
        type=int,
        help="Override the camera index in the tracker config.",
    )
    track_parser.add_argument(
        "--camera-source",
        help="Override the camera source URL, for example http://192.168.4.1:81/stream.",
    )
    track_parser.add_argument("--show-mask", action="store_true", help="Show the mask window.")
    track_parser.add_argument(
        "--hide-mask",
        action="store_true",
        help="Disable the mask window even if config enables it.",
    )
    track_parser.add_argument(
        "--no-uwb",
        action="store_true",
        help="Run vision only, without opening serial UWB anchors.",
    )
    track_parser.add_argument(
        "--no-auto-configure-anchors",
        action="store_true",
        help="Skip sending AT setup commands to the anchors on startup.",
    )
    track_parser.add_argument(
        "--print-json",
        action="store_true",
        help="Emit fused state as JSON lines on stdout.",
    )
    track_parser.set_defaults(func=run_track)

    return parser


def add_shared_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path, help="Load a saved detector config.")
    parser.add_argument(
        "--camera-index",
        type=int,
        help="Camera index to open. Defaults to the value in config or 0.",
    )
    parser.add_argument(
        "--camera-source",
        help="Camera URL or path, for example an ESP32 stream like http://192.168.4.1:81/stream.",
    )
    parser.add_argument("--frame-width", type=int, help="Requested camera width.")
    parser.add_argument("--frame-height", type=int, help="Requested camera height.")
    parser.add_argument("--value-max", type=int, help="Maximum HSV V value for black.")
    parser.add_argument(
        "--saturation-max",
        type=int,
        help="Maximum HSV S value for black.",
    )
    parser.add_argument(
        "--min-area",
        type=int,
        help="Ignore black blobs smaller than this many pixels.",
    )
    parser.add_argument("--show-mask", action="store_true", help="Show the mask window.")
    parser.add_argument(
        "--hide-mask",
        action="store_true",
        help="Disable the mask window even if config enables it.",
    )


def run_detect(args: argparse.Namespace) -> None:
    config = resolve_config(args)
    camera = open_camera(config)
    model = BlackColorModel(config)
    last_state: bool | None = None

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Failed to read a frame from the camera feed.")

            result = model.detect(frame)
            annotated = model.annotate(frame, result)

            if last_state is None or last_state != result.detected:
                status = "DETECTED" if result.detected else "NOT DETECTED"
                print(f"black_status={status} black_ratio={result.black_ratio:.4f}")
                last_state = result.detected

            cv2.imshow("Black Vision Detector", annotated)
            if config.show_mask:
                cv2.imshow("Black Mask", result.mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


def run_calibrate(args: argparse.Namespace) -> None:
    config = resolve_config(args)
    camera = open_camera(config)

    cv2.namedWindow("Calibration")
    cv2.resizeWindow("Calibration", 480, 220)
    cv2.createTrackbar("value_max", "Calibration", config.value_max, 255, _noop)
    cv2.createTrackbar("saturation_max", "Calibration", config.saturation_max, 255, _noop)
    cv2.createTrackbar("min_area", "Calibration", min(config.min_area, 20000), 20000, _noop)
    cv2.createTrackbar("blur_kernel", "Calibration", min(config.blur_kernel, 31), 31, _noop)
    cv2.createTrackbar("morph_kernel", "Calibration", min(config.morph_kernel, 31), 31, _noop)
    cv2.createTrackbar(
        "ratio_x100",
        "Calibration",
        int(config.black_ratio_threshold * 100),
        100,
        _noop,
    )

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Failed to read a frame from the camera feed.")

            live_config = DetectorConfig(
                camera_index=config.camera_index,
                camera_source=config.camera_source,
                frame_width=config.frame_width,
                frame_height=config.frame_height,
                value_max=cv2.getTrackbarPos("value_max", "Calibration"),
                saturation_max=cv2.getTrackbarPos("saturation_max", "Calibration"),
                min_area=max(1, cv2.getTrackbarPos("min_area", "Calibration")),
                blur_kernel=max(1, cv2.getTrackbarPos("blur_kernel", "Calibration")),
                morph_kernel=max(1, cv2.getTrackbarPos("morph_kernel", "Calibration")),
                black_ratio_threshold=cv2.getTrackbarPos("ratio_x100", "Calibration") / 100.0,
                show_mask=config.show_mask,
                draw_boxes=config.draw_boxes,
            ).normalized()

            model = BlackColorModel(live_config)
            result = model.detect(frame)
            annotated = model.annotate(frame, result)
            cv2.putText(
                annotated,
                "Press 's' to save, 'q' to quit",
                (20, annotated.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("Black Vision Detector", annotated)
            cv2.imshow("Black Mask", result.mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                save_config(live_config, args.output)
                print(f"Saved config to {args.output}")
                config = live_config
            if key == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


def run_track(args: argparse.Namespace) -> None:
    if args.write_default_tracker_config:
        config_path = save_tracker_config(TrackerConfig(), args.tracker_config)
        print(f"Wrote starter tracker config to {config_path}")
        return

    if not args.tracker_config.exists():
        raise FileNotFoundError(
            (
                f"{args.tracker_config} does not exist. "
                "Run `black-vision track --write-default-tracker-config` first."
            )
        )

    tracker_config = load_tracker_config(args.tracker_config)
    if args.camera_index is not None:
        tracker_config.detector.camera_index = args.camera_index
    if args.camera_source is not None:
        tracker_config.detector.camera_source = args.camera_source
    if args.show_mask:
        tracker_config.detector.show_mask = True
    if args.hide_mask:
        tracker_config.detector.show_mask = False
    tracker_config = tracker_config.normalized()

    tracking_system = DroneTrackingSystem(tracker_config)
    camera = open_camera(tracker_config.detector)

    anchors: list[tuple[object, ReyaxSerialAnchor]] = []
    next_poll_at: dict[str, float] = {}
    if not args.no_uwb:
        for anchor_config in tracker_config.anchors:
            anchor = ReyaxSerialAnchor(
                anchor_id=anchor_config.anchor_id,
                anchor_address=anchor_config.address,
                port=anchor_config.serial_port,
                baud_rate=tracker_config.serial_baud_rate,
            )
            anchor.open()
            if tracker_config.auto_configure_anchors and not args.no_auto_configure_anchors:
                anchor.configure(
                    network_id=tracker_config.uwb_network_id,
                    password=tracker_config.uwb_password,
                    channel=tracker_config.uwb_channel,
                    bandwidth=tracker_config.uwb_bandwidth,
                    enable_rssi=tracker_config.enable_rssi,
                )
            anchors.append((anchor_config, anchor))
            next_poll_at[anchor_config.anchor_id] = time.monotonic()

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Failed to read a frame from the camera feed.")

            now = time.monotonic()
            measurements = []
            for anchor_config, anchor in anchors:
                if now < next_poll_at[anchor_config.anchor_id]:
                    continue
                measurement = anchor.poll_distance(
                    tag_address=tracker_config.drone_tag.address,
                    payload=tracker_config.anchor_poll_payload,
                    response_timeout_s=tracker_config.anchor_response_timeout_s,
                )
                next_poll_at[anchor_config.anchor_id] = now + anchor_config.poll_interval_s
                if measurement is not None:
                    measurements.append(measurement)

            result = tracking_system.process(frame, measurements=measurements, timestamp_s=now)
            annotated = tracking_system.annotate(frame, result)

            if args.print_json and (measurements or result.estimate is not None):
                print(json.dumps(result.to_dict()))

            cv2.imshow("Black Drone Tracker", annotated)
            if tracker_config.detector.show_mask:
                cv2.imshow("Black Mask", result.detection.mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        camera.release()
        for _, anchor in anchors:
            anchor.close()
        cv2.destroyAllWindows()


def resolve_config(args: argparse.Namespace) -> DetectorConfig:
    config = load_config(args.config) if args.config else DetectorConfig()

    if args.camera_index is not None:
        config.camera_index = args.camera_index
    if args.camera_source is not None:
        config.camera_source = args.camera_source
    if args.frame_width is not None:
        config.frame_width = args.frame_width
    if args.frame_height is not None:
        config.frame_height = args.frame_height
    if args.value_max is not None:
        config.value_max = args.value_max
    if args.saturation_max is not None:
        config.saturation_max = args.saturation_max
    if args.min_area is not None:
        config.min_area = args.min_area
    if args.show_mask:
        config.show_mask = True
    if args.hide_mask:
        config.show_mask = False

    return config.normalized()


def open_camera(config: DetectorConfig) -> cv2.VideoCapture:
    if config.camera_source:
        camera = cv2.VideoCapture(config.camera_source)
        if not camera.isOpened():
            raise RuntimeError(
                (
                    f"Could not open camera source {config.camera_source}. "
                    "For ESP32-CAM try a stream URL like http://<ip>:81/stream."
                )
            )
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
        return camera

    backends = []
    if hasattr(cv2, "CAP_AVFOUNDATION"):
        backends.append(cv2.CAP_AVFOUNDATION)
    backends.append(cv2.CAP_ANY)

    for backend in backends:
        camera = cv2.VideoCapture(config.camera_index, backend)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
            return camera
        camera.release()

    raise RuntimeError(
        (
            f"Could not open camera index {config.camera_index}. "
            "If you are using ESP32-CAM, pass --camera-source http://<ip>:81/stream."
        )
    )


def _noop(_: int) -> None:
    return None


if __name__ == "__main__":
    main()
