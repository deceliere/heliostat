#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import paho.mqtt.client as mqtt


BASE_DIR = Path(__file__).resolve().parent.parent
DATA_DIR = BASE_DIR / "data"
DEFAULT_REFERENCE_PATH = DATA_DIR / "vision_reference.json"
DEFAULT_ZONE_PATH = DATA_DIR / "vision_zone.json"


@dataclass
class ReferencePoint:
    x: float
    y: float


@dataclass
class SpotMeasurement:
    x: float | None
    y: float | None
    area_px: float
    confidence: float
    max_value: float
    threshold_value: float
    dx_px: float | None
    dy_px: float | None
    distance_px: float | None


@dataclass
class ZoneRect:
    x0: float
    y0: float
    x1: float
    y1: float

    @staticmethod
    def from_points(x0: float, y0: float, x1: float, y1: float) -> ZoneRect:
        return ZoneRect(
            x0=min(x0, x1),
            y0=min(y0, y1),
            x1=max(x0, x1),
            y1=max(y0, y1),
        )


@dataclass
class TrackingState:
    anchor: ReferencePoint | None = None
    missed_frames: int = 0


class MqttPublisher:
    def __init__(self, host: str, port: int, base_topic: str, client_id: str) -> None:
        self._availability_topic = f"{base_topic}/vision/availability"
        self._state_topic = f"{base_topic}/vision/state"
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._connected = False
        self._host = host
        self._port = port
        self._last_connect_attempt_monotonic = 0.0

    def _on_connect(self, _client: mqtt.Client, _userdata: Any, _flags: Any, reason_code: Any, _properties: Any) -> None:
        self._connected = int(getattr(reason_code, "value", reason_code)) == 0
        if self._connected:
            self._client.publish(self._availability_topic, "online", qos=0, retain=True)
            print(f"MQTT connected to {self._host}:{self._port}")
        else:
            print(f"MQTT connect failed, reason={reason_code}")

    def _on_disconnect(self, _client: mqtt.Client, _userdata: Any, _disconnect_flags: Any, reason_code: Any, _properties: Any) -> None:
        self._connected = False
        print(f"MQTT disconnected, reason={reason_code}")

    def ensure_connected(self) -> None:
        if self._connected:
            return
        now = time.monotonic()
        if now - self._last_connect_attempt_monotonic < 3.0:
            return
        self._last_connect_attempt_monotonic = now
        try:
            print(f"MQTT connect: {self._host}:{self._port}")
            self._client.connect(self._host, self._port, keepalive=30)
        except OSError as exc:
            print(f"MQTT connect error: {exc}")

    def loop(self) -> None:
        self._client.loop(timeout=0.0)

    def publish_state(self, payload: dict[str, Any]) -> None:
        if not self._connected:
            return
        self._client.publish(self._state_topic, json.dumps(payload), qos=0, retain=False)

    def close(self) -> None:
        if self._connected:
            self._client.publish(self._availability_topic, "offline", qos=0, retain=True)
        try:
            self._client.disconnect()
        except OSError:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Track a bright heliostat spot from an RTMP stream.")
    parser.add_argument("--stream-url", default=os.getenv("HELIOSTAT_VISION_STREAM_URL", ""), help="RTMP stream URL")
    parser.add_argument("--mqtt-host", default=os.getenv("HELIOSTAT_MQTT_HOST", "127.0.0.1"))
    parser.add_argument("--mqtt-port", type=int, default=int(os.getenv("HELIOSTAT_MQTT_PORT", "1883")))
    parser.add_argument("--mqtt-base-topic", default=os.getenv("HELIOSTAT_MQTT_BASE_TOPIC", "heliostat/remote1"))
    parser.add_argument("--publish-mqtt", action="store_true", help="Publish vision state to MQTT")
    parser.add_argument("--display", action="store_true", help="Show a live debug window")
    parser.add_argument("--show-mask", action="store_true", help="Show the threshold mask in a second window")
    parser.add_argument("--publish-interval-ms", type=int, default=200)
    parser.add_argument("--fps-limit", type=float, default=10.0)
    parser.add_argument("--resize-width", type=int, default=0, help="Resize frames before detection/display; 0 keeps the source size")
    parser.add_argument("--contrast", type=float, default=1.0, help="Linear contrast factor applied before detection")
    parser.add_argument("--brightness", type=int, default=0, help="Brightness offset applied before detection")
    parser.add_argument("--min-threshold", type=int, default=220)
    parser.add_argument("--threshold-drop", type=int, default=25)
    parser.add_argument("--min-area-px", type=float, default=8.0)
    parser.add_argument("--roi-radius-px", type=int, default=180, help="0 disables ROI cropping around the reference")
    parser.add_argument("--max-lock-distance-px", type=float, default=120.0, help="Reject blobs too far from the tracking anchor; 0 disables")
    parser.add_argument("--sticky-missed-frames", type=int, default=12, help="How long to keep the last anchor after losing the spot")
    parser.add_argument("--blur-kernel", type=int, default=5)
    parser.add_argument("--open-kernel", type=int, default=3)
    parser.add_argument("--client-id", default="heliostat-vision")
    parser.add_argument("--reference-file", default=str(DEFAULT_REFERENCE_PATH))
    parser.add_argument("--zone-file", default=str(DEFAULT_ZONE_PATH))
    return parser


def load_reference(path: Path) -> ReferencePoint | None:
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    x = payload.get("x")
    y = payload.get("y")
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return None
    return ReferencePoint(float(x), float(y))


def save_reference(path: Path, reference: ReferencePoint | None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if reference is None:
        path.write_text("null\n", encoding="utf-8")
        return
    payload = {"x": reference.x, "y": reference.y}
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def load_zone(path: Path) -> ZoneRect | None:
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    values = [payload.get(key) for key in ("x0", "y0", "x1", "y1")]
    if not all(isinstance(value, (int, float)) for value in values):
        return None
    return ZoneRect.from_points(float(values[0]), float(values[1]), float(values[2]), float(values[3]))


def save_zone(path: Path, zone: ZoneRect | None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if zone is None:
        path.write_text("null\n", encoding="utf-8")
        return
    payload = {"x0": zone.x0, "y0": zone.y0, "x1": zone.x1, "y1": zone.y1}
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def compute_roi(frame: np.ndarray, center: ReferencePoint | None, radius_px: int) -> tuple[np.ndarray, tuple[int, int]]:
    if center is None or radius_px <= 0:
        return frame, (0, 0)
    height, width = frame.shape[:2]
    cx = int(round(center.x))
    cy = int(round(center.y))
    x0 = max(0, cx - radius_px)
    y0 = max(0, cy - radius_px)
    x1 = min(width, cx + radius_px)
    y1 = min(height, cy + radius_px)
    return frame[y0:y1, x0:x1], (x0, y0)


def preprocess_frame(frame: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    contrast = max(0.1, float(args.contrast))
    brightness = int(args.brightness)
    if abs(contrast - 1.0) < 1e-6 and brightness == 0:
        return frame
    return cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)


def resize_frame_if_needed(frame: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    target_width = max(0, int(args.resize_width))
    if target_width <= 0:
        return frame

    height, width = frame.shape[:2]
    if width <= target_width:
        return frame

    scale = target_width / float(width)
    target_height = max(1, int(round(height * scale)))
    return cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)


def apply_zone_mask(mask: np.ndarray, zone: ZoneRect | None, offset_x: int, offset_y: int) -> np.ndarray:
    if zone is None:
        return mask

    height, width = mask.shape[:2]
    local_x0 = max(0, int(round(zone.x0)) - offset_x)
    local_y0 = max(0, int(round(zone.y0)) - offset_y)
    local_x1 = min(width, int(round(zone.x1)) - offset_x)
    local_y1 = min(height, int(round(zone.y1)) - offset_y)

    if local_x1 <= local_x0 or local_y1 <= local_y0:
        return np.zeros_like(mask)

    zone_mask = np.zeros_like(mask)
    zone_mask[local_y0:local_y1, local_x0:local_x1] = 255
    return cv2.bitwise_and(mask, zone_mask)


def detect_spot(
    frame: np.ndarray,
    reference: ReferencePoint | None,
    tracking_anchor: ReferencePoint | None,
    zone: ZoneRect | None,
    args: argparse.Namespace,
) -> tuple[SpotMeasurement, np.ndarray]:
    roi_center = tracking_anchor if tracking_anchor is not None else reference
    roi, (offset_x, offset_y) = compute_roi(frame, roi_center, args.roi_radius_px)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    blur_kernel = max(1, int(args.blur_kernel))
    if blur_kernel % 2 == 0:
        blur_kernel += 1
    blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)

    min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(blurred)
    threshold_value = max(args.min_threshold, int(max_val) - int(args.threshold_drop))
    _, mask = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)
    mask = apply_zone_mask(mask, zone, offset_x, offset_y)

    open_kernel = max(1, int(args.open_kernel))
    kernel = np.ones((open_kernel, open_kernel), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    best_label = -1
    best_sort_key: tuple[float, float, float] | None = None
    best_centroid: tuple[float, float] | None = None
    best_area = 0.0

    for label in range(1, num_labels):
        area_px = float(stats[label, cv2.CC_STAT_AREA])
        if area_px < args.min_area_px:
            continue
        centroid_x, centroid_y = centroids[label]
        centroid_abs_x = float(centroid_x + offset_x)
        centroid_abs_y = float(centroid_y + offset_y)
        mean_score = float(blurred[labels == label].mean()) if np.any(labels == label) else 0.0
        distance_to_anchor = None
        if tracking_anchor is not None:
            distance_to_anchor = math.hypot(centroid_abs_x - tracking_anchor.x, centroid_abs_y - tracking_anchor.y)
            if args.max_lock_distance_px > 0 and distance_to_anchor > args.max_lock_distance_px:
                continue

        if tracking_anchor is not None:
            sort_key = (
                distance_to_anchor if distance_to_anchor is not None else float("inf"),
                -mean_score,
                -area_px,
            )
        elif reference is not None:
            distance_to_reference = math.hypot(centroid_abs_x - reference.x, centroid_abs_y - reference.y)
            sort_key = (distance_to_reference, -mean_score, -area_px)
        else:
            sort_key = (0.0, -mean_score, -area_px)

        if best_sort_key is None or sort_key < best_sort_key:
            best_sort_key = sort_key
            best_label = label
            best_centroid = (centroid_abs_x, centroid_abs_y)
            best_area = area_px

    if best_label < 0 or best_centroid is None:
        return (
            SpotMeasurement(
                x=None,
                y=None,
                area_px=0.0,
                confidence=0.0,
                max_value=float(max_val),
                threshold_value=float(threshold_value),
                dx_px=None,
                dy_px=None,
                distance_px=None,
            ),
            mask,
        )

    dx_px = None
    dy_px = None
    distance_px = None
    if reference is not None:
        dx_px = best_centroid[0] - reference.x
        dy_px = best_centroid[1] - reference.y
        distance_px = math.hypot(dx_px, dy_px)

    mean_score = float(blurred[labels == best_label].mean()) if np.any(labels == best_label) else 0.0
    confidence = min(1.0, max(0.0, mean_score / 255.0))
    return (
        SpotMeasurement(
            x=best_centroid[0],
            y=best_centroid[1],
            area_px=best_area,
            confidence=confidence,
            max_value=float(max_val),
            threshold_value=float(threshold_value),
            dx_px=dx_px,
            dy_px=dy_px,
            distance_px=distance_px,
        ),
        mask,
    )


def draw_overlay(
    frame: np.ndarray,
    reference: ReferencePoint | None,
    tracking_anchor: ReferencePoint | None,
    zone: ZoneRect | None,
    dragging_zone: ZoneRect | None,
    measurement: SpotMeasurement,
    fps: float,
    missed_frames: int,
    zone_edit_mode: bool,
    args: argparse.Namespace,
) -> np.ndarray:
    annotated = frame.copy()

    if reference is not None:
        ref_point = (int(round(reference.x)), int(round(reference.y)))
        cv2.drawMarker(annotated, ref_point, (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
        cv2.circle(annotated, ref_point, 24, (0, 120, 0), 1)

    if tracking_anchor is not None:
        anchor_point = (int(round(tracking_anchor.x)), int(round(tracking_anchor.y)))
        cv2.circle(annotated, anchor_point, 18, (255, 160, 0), 1)

    if zone is not None:
        cv2.rectangle(
            annotated,
            (int(round(zone.x0)), int(round(zone.y0))),
            (int(round(zone.x1)), int(round(zone.y1))),
            (0, 255, 255),
            2,
        )

    if dragging_zone is not None:
        cv2.rectangle(
            annotated,
            (int(round(dragging_zone.x0)), int(round(dragging_zone.y0))),
            (int(round(dragging_zone.x1)), int(round(dragging_zone.y1))),
            (255, 0, 255),
            2,
        )

    if measurement.x is not None and measurement.y is not None:
        spot_point = (int(round(measurement.x)), int(round(measurement.y)))
        cv2.circle(annotated, spot_point, 10, (0, 0, 255), 2)
        cv2.drawMarker(annotated, spot_point, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
        if reference is not None:
            ref_point = (int(round(reference.x)), int(round(reference.y)))
            cv2.line(annotated, ref_point, spot_point, (0, 180, 255), 2)

    lines = [
        f"FPS: {fps:.1f}",
        f"Spot: {measurement.x:.1f}, {measurement.y:.1f}" if measurement.x is not None else "Spot: not found",
        f"Anchor misses: {missed_frames}",
        f"dX/dY: {measurement.dx_px:.1f}, {measurement.dy_px:.1f}" if measurement.dx_px is not None else "dX/dY: reference not set",
        f"dist/conf: {measurement.distance_px:.1f}px / {measurement.confidence:.2f}" if measurement.distance_px is not None else f"conf: {measurement.confidence:.2f}",
        f"thr/max: {measurement.threshold_value:.0f} / {measurement.max_value:.0f}",
        f"frame: {frame.shape[1]}x{frame.shape[0]}",
        f"contrast/brightness: {args.contrast:.2f} / {args.brightness:+d}",
        f"zone mode: {'ON' if zone_edit_mode else 'off'}",
        "left click = set ref | shift+drag or z+drag = set zone | c = clear ref | x = clear zone | z = toggle zone mode | q = quit",
    ]
    y = 24
    for line in lines:
        if line:
            cv2.putText(annotated, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            y += 22

    if zone_edit_mode:
        cv2.putText(annotated, "ZONE EDIT MODE", (12, y + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)

    return annotated


def build_payload(
    reference: ReferencePoint | None,
    zone: ZoneRect | None,
    measurement: SpotMeasurement,
    frame: np.ndarray,
    stream_ok: bool,
) -> dict[str, Any]:
    height, width = frame.shape[:2]
    return {
        "stream_ok": stream_ok,
        "frame_width_px": width,
        "frame_height_px": height,
        "reference_x_px": reference.x if reference is not None else None,
        "reference_y_px": reference.y if reference is not None else None,
        "zone_x0_px": zone.x0 if zone is not None else None,
        "zone_y0_px": zone.y0 if zone is not None else None,
        "zone_x1_px": zone.x1 if zone is not None else None,
        "zone_y1_px": zone.y1 if zone is not None else None,
        "spot_x_px": measurement.x,
        "spot_y_px": measurement.y,
        "dx_px": measurement.dx_px,
        "dy_px": measurement.dy_px,
        "distance_px": measurement.distance_px,
        "area_px": measurement.area_px,
        "confidence": measurement.confidence,
        "threshold_value": measurement.threshold_value,
        "max_value": measurement.max_value,
        "timestamp_unix_ms": int(round(time.time() * 1000)),
    }


def main() -> int:
    args = build_parser().parse_args()
    if not args.stream_url:
        print("Missing stream URL. Set --stream-url or HELIOSTAT_VISION_STREAM_URL.")
        return 2

    reference_path = Path(args.reference_file)
    reference = load_reference(reference_path)
    zone_path = Path(args.zone_file)
    zone = load_zone(zone_path)
    tracking_state = TrackingState(anchor=reference)
    if reference is not None:
        print(f"Loaded reference: x={reference.x:.1f} y={reference.y:.1f}")
    if zone is not None:
        print(f"Loaded zone: x0={zone.x0:.1f} y0={zone.y0:.1f} x1={zone.x1:.1f} y1={zone.y1:.1f}")

    publisher = MqttPublisher(args.mqtt_host, args.mqtt_port, args.mqtt_base_topic, args.client_id) if args.publish_mqtt else None

    video = cv2.VideoCapture(args.stream_url)
    if not video.isOpened():
        print(f"Failed to open stream: {args.stream_url}")
        return 3
    video.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    window_name = "heliostat spot tracker"
    mask_window_name = "heliostat spot mask"

    running = True

    def stop_handler(_signum: int, _frame: Any) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    if args.display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        drag_start: tuple[int, int] | None = None
        drag_current: tuple[int, int] | None = None
        zone_edit_mode = False

        def on_mouse(event: int, x: int, y: int, _flags: int, _userdata: Any) -> None:
            nonlocal reference, zone, drag_start, drag_current, zone_edit_mode
            shift_pressed = bool(_flags & cv2.EVENT_FLAG_SHIFTKEY)
            drawing_zone = shift_pressed or zone_edit_mode

            if event == cv2.EVENT_LBUTTONDOWN and drawing_zone:
                drag_start = (x, y)
                drag_current = (x, y)
                return

            if event == cv2.EVENT_LBUTTONDOWN:
                reference = ReferencePoint(float(x), float(y))
                tracking_state.anchor = reference
                tracking_state.missed_frames = 0
                save_reference(reference_path, reference)
                print(f"Reference set: x={x} y={y}")
                return

            if event == cv2.EVENT_MOUSEMOVE and drag_start is not None:
                drag_current = (x, y)
                return

            if event == cv2.EVENT_LBUTTONUP and drag_start is not None:
                drag_current = (x, y)
                zone = ZoneRect.from_points(float(drag_start[0]), float(drag_start[1]), float(drag_current[0]), float(drag_current[1]))
                save_zone(zone_path, zone)
                print(f"Zone set: x0={zone.x0:.0f} y0={zone.y0:.0f} x1={zone.x1:.0f} y1={zone.y1:.0f}")
                drag_start = None
                drag_current = None

        cv2.setMouseCallback(window_name, on_mouse)
        if args.show_mask:
            cv2.namedWindow(mask_window_name, cv2.WINDOW_NORMAL)

    last_publish_monotonic = 0.0
    last_frame_monotonic = time.monotonic()
    fps = 0.0

    while running:
        if publisher is not None:
            publisher.ensure_connected()
            publisher.loop()

        ok, frame = video.read()
        if not ok or frame is None:
            print("Stream read failed, retrying...")
            time.sleep(0.2)
            continue

        frame = resize_frame_if_needed(frame, args)

        now = time.monotonic()
        dt = now - last_frame_monotonic
        last_frame_monotonic = now
        if dt > 0.0:
            fps = 1.0 / dt

        processed_frame = preprocess_frame(frame, args)
        measurement, mask = detect_spot(processed_frame, reference, tracking_state.anchor, zone, args)
        if measurement.x is not None and measurement.y is not None:
            tracking_state.anchor = ReferencePoint(measurement.x, measurement.y)
            tracking_state.missed_frames = 0
        else:
            tracking_state.missed_frames += 1
            if tracking_state.missed_frames > args.sticky_missed_frames:
                tracking_state.anchor = reference

        payload = build_payload(reference, zone, measurement, frame, stream_ok=True)

        if publisher is not None and (now - last_publish_monotonic) * 1000.0 >= args.publish_interval_ms:
            publisher.publish_state(payload)
            last_publish_monotonic = now

        if args.display:
            dragging_zone = None
            if drag_start is not None and drag_current is not None:
                dragging_zone = ZoneRect.from_points(float(drag_start[0]), float(drag_start[1]), float(drag_current[0]), float(drag_current[1]))
            cv2.imshow(
                window_name,
                draw_overlay(
                    processed_frame,
                    reference,
                    tracking_state.anchor,
                    zone,
                    dragging_zone,
                    measurement,
                    fps,
                    tracking_state.missed_frames,
                    zone_edit_mode,
                    args,
                ),
            )
            if args.show_mask:
                cv2.imshow(mask_window_name, mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                reference = None
                tracking_state.anchor = None
                tracking_state.missed_frames = 0
                save_reference(reference_path, None)
                print("Reference cleared")
            if key == ord("x"):
                zone = None
                save_zone(zone_path, None)
                print("Zone cleared")
            if key == ord("z"):
                zone_edit_mode = not zone_edit_mode
                print(f"Zone edit mode {'enabled' if zone_edit_mode else 'disabled'}")

        if args.fps_limit > 0:
            target_period = 1.0 / args.fps_limit
            remaining = target_period - (time.monotonic() - now)
            if remaining > 0:
                time.sleep(remaining)

    video.release()
    if args.display:
        cv2.destroyAllWindows()
    if publisher is not None:
        publisher.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
