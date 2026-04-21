#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


@dataclass
class DriftRow:
    timestamp_unix_ms: int
    label: str
    pan_correction_deg: float
    tilt_correction_deg: float
    sun_bearing_deg: float
    sun_elevation_deg: float
    target_bearing_deg: float
    target_elevation_deg: float
    pan_deg: float
    tilt_deg: float
    pan_target_deg: float
    tilt_target_deg: float
    predicted_pan_deg: float
    predicted_tilt_deg: float
    pan_tracking_error_deg: float
    tilt_tracking_error_deg: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze heliostat drift samples exported from the UI."
    )
    parser.add_argument("json_file", type=Path, help="Path to exported drift JSON")
    parser.add_argument(
        "--table",
        action="store_true",
        help="Print one compact line per sample",
    )
    return parser.parse_args()


def require_number(value: object, field: str) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    raise ValueError(f"Missing or invalid numeric field: {field}")


def load_rows(path: Path) -> list[DriftRow]:
    payload = json.loads(path.read_text())
    if not isinstance(payload, list):
        raise ValueError("Top-level JSON value must be a list")

    rows: list[DriftRow] = []
    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"Sample {index} is not an object")
        state = item.get("state")
        if not isinstance(state, dict):
            raise ValueError(f"Sample {index} has no valid state object")

        rows.append(
            DriftRow(
                timestamp_unix_ms=int(require_number(item.get("timestamp_unix_ms"), "timestamp_unix_ms")),
                label=str(item.get("label") or ""),
                pan_correction_deg=require_number(item.get("pan_correction_deg"), "pan_correction_deg"),
                tilt_correction_deg=require_number(item.get("tilt_correction_deg"), "tilt_correction_deg"),
                sun_bearing_deg=require_number(state.get("sun_bearing_deg"), "state.sun_bearing_deg"),
                sun_elevation_deg=require_number(state.get("sun_elevation_deg"), "state.sun_elevation_deg"),
                target_bearing_deg=require_number(state.get("target_bearing_deg"), "state.target_bearing_deg"),
                target_elevation_deg=require_number(state.get("target_elevation_deg"), "state.target_elevation_deg"),
                pan_deg=require_number(state.get("pan_deg"), "state.pan_deg"),
                tilt_deg=require_number(state.get("tilt_deg"), "state.tilt_deg"),
                pan_target_deg=require_number(state.get("pan_target_deg"), "state.pan_target_deg"),
                tilt_target_deg=require_number(state.get("tilt_target_deg"), "state.tilt_target_deg"),
                predicted_pan_deg=require_number(state.get("predicted_pan_deg"), "state.predicted_pan_deg"),
                predicted_tilt_deg=require_number(state.get("predicted_tilt_deg"), "state.predicted_tilt_deg"),
                pan_tracking_error_deg=require_number(state.get("pan_tracking_error_deg"), "state.pan_tracking_error_deg"),
                tilt_tracking_error_deg=require_number(state.get("tilt_tracking_error_deg"), "state.tilt_tracking_error_deg"),
            )
        )
    rows.sort(key=lambda row: row.timestamp_unix_ms)
    return rows


def fmt_ts(unix_ms: int) -> str:
    dt = datetime.fromtimestamp(unix_ms / 1000.0, tz=timezone.utc)
    return dt.isoformat().replace("+00:00", "Z")


def mean(values: Iterable[float]) -> float:
    items = list(values)
    return sum(items) / len(items) if items else math.nan


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    if len(xs) != len(ys) or len(xs) < 2:
        return math.nan, math.nan, math.nan

    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    var_x = sum((x - mean_x) ** 2 for x in xs)
    var_y = sum((y - mean_y) ** 2 for y in ys)
    if var_x == 0 or var_y == 0:
        return math.nan, mean_y, math.nan

    cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = cov / var_x
    intercept = mean_y - slope * mean_x
    correlation = cov / math.sqrt(var_x * var_y)
    return slope, intercept, correlation


def print_summary(rows: list[DriftRow]) -> None:
    first = rows[0]
    last = rows[-1]
    duration_hours = (last.timestamp_unix_ms - first.timestamp_unix_ms) / 3_600_000.0

    target_bearing_values = [row.target_bearing_deg for row in rows]
    target_elevation_values = [row.target_elevation_deg for row in rows]
    servo_pan_error = [row.pan_deg - row.pan_target_deg for row in rows]
    servo_tilt_error = [row.tilt_deg - row.tilt_target_deg for row in rows]
    model_pan_error = [row.pan_tracking_error_deg for row in rows]
    model_tilt_error = [row.tilt_tracking_error_deg for row in rows]

    elapsed_minutes = [
        (row.timestamp_unix_ms - first.timestamp_unix_ms) / 60_000.0 for row in rows
    ]
    pan_corr = [row.pan_correction_deg for row in rows]
    tilt_corr = [row.tilt_correction_deg for row in rows]
    sun_bearing = [row.sun_bearing_deg for row in rows]
    sun_elevation = [row.sun_elevation_deg for row in rows]

    pan_time_slope, _, pan_time_r = linear_fit(elapsed_minutes, pan_corr)
    tilt_time_slope, _, tilt_time_r = linear_fit(elapsed_minutes, tilt_corr)
    pan_bearing_slope, _, pan_bearing_r = linear_fit(sun_bearing, pan_corr)
    tilt_elev_slope, _, tilt_elev_r = linear_fit(sun_elevation, tilt_corr)

    print("Drift analysis")
    print(f"- Samples: {len(rows)}")
    print(f"- Time range: {fmt_ts(first.timestamp_unix_ms)} -> {fmt_ts(last.timestamp_unix_ms)}")
    print(f"- Duration: {duration_hours:.2f} h")
    print(
        f"- Captured target stability: bearing span "
        f"{max(target_bearing_values) - min(target_bearing_values):.4f}°, elevation span "
        f"{max(target_elevation_values) - min(target_elevation_values):.4f}°"
    )
    print(
        f"- Servo following command: mean(actual-target) pan {mean(servo_pan_error):+.3f}°, "
        f"tilt {mean(servo_tilt_error):+.3f}°"
    )
    print(
        f"- Model residual: mean(predicted-actual) pan {mean(model_pan_error):+.3f}°, "
        f"tilt {mean(model_tilt_error):+.3f}°"
    )
    print(
        f"- Manual correction trend vs time: pan {pan_time_slope:+.4f}°/min (r={pan_time_r:+.3f}), "
        f"tilt {tilt_time_slope:+.4f}°/min (r={tilt_time_r:+.3f})"
    )
    print(
        f"- Manual pan correction vs sun bearing: {pan_bearing_slope:+.4f}°/bearing° (r={pan_bearing_r:+.3f})"
    )
    print(
        f"- Manual tilt correction vs sun elevation: {tilt_elev_slope:+.4f}°/elevation° (r={tilt_elev_r:+.3f})"
    )


def print_table(rows: list[DriftRow]) -> None:
    print()
    print(
        "time_utc".ljust(21),
        "sun".ljust(15),
        "corr".ljust(17),
        "actual".ljust(17),
        "predicted".ljust(17),
        "residual".ljust(17),
    )
    for row in rows:
        print(
            fmt_ts(row.timestamp_unix_ms).ljust(21),
            f"{row.sun_bearing_deg:6.1f}/{row.sun_elevation_deg:6.1f}".ljust(15),
            f"{row.pan_correction_deg:+6.2f}/{row.tilt_correction_deg:+6.2f}".ljust(17),
            f"{row.pan_deg:6.2f}/{row.tilt_deg:6.2f}".ljust(17),
            f"{row.predicted_pan_deg:6.2f}/{row.predicted_tilt_deg:6.2f}".ljust(17),
            f"{row.pan_tracking_error_deg:+6.2f}/{row.tilt_tracking_error_deg:+6.2f}".ljust(17),
        )


def main() -> int:
    args = parse_args()
    rows = load_rows(args.json_file)
    if not rows:
        raise ValueError("No samples found")

    print_summary(rows)
    if args.table:
        print_table(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
