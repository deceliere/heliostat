#!/usr/bin/env python3
from __future__ import annotations

import argparse
import html
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


@dataclass
class DriftRow:
    timestamp_unix_ms: int
    baseline_timestamp_unix_ms: int | None
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
    parser.add_argument(
        "--svg",
        nargs="?",
        const="",
        metavar="OUTPUT",
        help="Write a self-contained SVG graph. Default: next to JSON with .svg extension",
    )
    return parser.parse_args()


def require_number(value: object, field: str) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    raise ValueError(f"Missing or invalid numeric field: {field}")


def first_present(mapping: dict, *keys: str) -> object:
    for key in keys:
        value = mapping.get(key)
        if value is not None:
            return value
    return None


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
        baseline = item.get("baseline")
        baseline_timestamp_unix_ms = None
        if isinstance(baseline, dict) and isinstance(baseline.get("timestamp_unix_ms"), (int, float)):
            baseline_timestamp_unix_ms = int(baseline["timestamp_unix_ms"])

        rows.append(
            DriftRow(
                timestamp_unix_ms=int(require_number(item.get("timestamp_unix_ms"), "timestamp_unix_ms")),
                baseline_timestamp_unix_ms=baseline_timestamp_unix_ms,
                label=str(item.get("label") or ""),
                pan_correction_deg=require_number(item.get("pan_correction_deg"), "pan_correction_deg"),
                tilt_correction_deg=require_number(item.get("tilt_correction_deg"), "tilt_correction_deg"),
                sun_bearing_deg=require_number(state.get("sun_bearing_deg"), "state.sun_bearing_deg"),
                sun_elevation_deg=require_number(state.get("sun_elevation_deg"), "state.sun_elevation_deg"),
                target_bearing_deg=require_number(
                    first_present(state, "beam_target_bearing_deg", "target_bearing_deg"),
                    "state.beam_target_bearing_deg|state.target_bearing_deg",
                ),
                target_elevation_deg=require_number(
                    first_present(state, "beam_target_elevation_deg", "target_elevation_deg"),
                    "state.beam_target_elevation_deg|state.target_elevation_deg",
                ),
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


def fmt_report_stamp(unix_ms: int) -> str:
    dt = datetime.fromtimestamp(unix_ms / 1000.0, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H-%M-%SZ")


def fmt_hms_from_unix_seconds(unix_seconds: float) -> str:
    dt = datetime.fromtimestamp(unix_seconds, tz=timezone.utc)
    return dt.strftime("%H:%M:%S")


def first_baseline_unix_ms(rows: list[DriftRow]) -> int:
    baseline_values = [
        row.baseline_timestamp_unix_ms
        for row in rows
        if row.baseline_timestamp_unix_ms is not None
    ]
    if baseline_values:
        return min(baseline_values)
    return rows[0].timestamp_unix_ms


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


def value_range(series_list: list[list[float]]) -> tuple[float, float]:
    values = [value for series in series_list for value in series if math.isfinite(value)]
    if not values:
        return -1.0, 1.0
    minimum = min(values)
    maximum = max(values)
    if math.isclose(minimum, maximum, abs_tol=1e-9):
        padding = 1.0 if math.isclose(minimum, 0.0, abs_tol=1e-9) else abs(minimum) * 0.2
        return minimum - padding, maximum + padding
    padding = max(0.2, (maximum - minimum) * 0.12)
    return minimum - padding, maximum + padding


def scale_point(
    x_value: float,
    y_value: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    plot_x: float,
    plot_y: float,
    plot_width: float,
    plot_height: float,
) -> tuple[float, float]:
    x_ratio = 0.0 if math.isclose(x_min, x_max) else (x_value - x_min) / (x_max - x_min)
    y_ratio = 0.0 if math.isclose(y_min, y_max) else (y_value - y_min) / (y_max - y_min)
    x = plot_x + (x_ratio * plot_width)
    y = plot_y + plot_height - (y_ratio * plot_height)
    return x, y


def polyline_points(
    xs: list[float],
    ys: list[float],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    plot_x: float,
    plot_y: float,
    plot_width: float,
    plot_height: float,
) -> str:
    points: list[str] = []
    for x_value, y_value in zip(xs, ys):
        x, y = scale_point(
            x_value, y_value, x_min, x_max, y_min, y_max, plot_x, plot_y, plot_width, plot_height
        )
        points.append(f"{x:.1f},{y:.1f}")
    return " ".join(points)


def fit_line_points(
    xs: list[float],
    ys: list[float],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    plot_x: float,
    plot_y: float,
    plot_width: float,
    plot_height: float,
) -> str:
    slope, intercept, _ = linear_fit(xs, ys)
    if not math.isfinite(slope) or not math.isfinite(intercept):
        return ""
    x0, y0 = scale_point(
        x_min, slope * x_min + intercept, x_min, x_max, y_min, y_max, plot_x, plot_y, plot_width, plot_height
    )
    x1, y1 = scale_point(
        x_max, slope * x_max + intercept, x_min, x_max, y_min, y_max, plot_x, plot_y, plot_width, plot_height
    )
    return f"{x0:.1f},{y0:.1f} {x1:.1f},{y1:.1f}"


def scatter_points_svg(
    xs: list[float],
    ys: list[float],
    color: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    plot_x: float,
    plot_y: float,
    plot_width: float,
    plot_height: float,
) -> str:
    circles: list[str] = []
    for x_value, y_value in zip(xs, ys):
        x, y = scale_point(
            x_value, y_value, x_min, x_max, y_min, y_max, plot_x, plot_y, plot_width, plot_height
        )
        circles.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.2" fill="{color}" stroke="#0f1726" stroke-width="1.0"/>'
        )
    return "\n".join(circles)


def chart_svg(
    title: str,
    x_label: str,
    y_label: str,
    xs: list[float],
    series: list[tuple[str, list[float], str]],
    top_y: float,
) -> str:
    width = 1020
    height = 320
    left = 72
    right = 24
    top = top_y + 34
    bottom = 52
    plot_width = width - left - right
    plot_height = height - 72
    x_min = min(xs) if xs else 0.0
    x_max = max(xs) if xs else 1.0
    y_min, y_max = value_range([values for _, values, _ in series])

    y_ticks = 5
    x_ticks = 6
    parts = [
        f'<text x="{left}" y="{top_y + 18:.1f}" font-size="16" font-weight="600" fill="#dbe4ff">{html.escape(title)}</text>',
        f'<rect x="{left}" y="{top:.1f}" width="{plot_width:.1f}" height="{plot_height:.1f}" rx="10" fill="#08111f" stroke="#23324a"/>',
    ]

    for index in range(y_ticks + 1):
        ratio = index / y_ticks
        y_value = y_min + ((y_max - y_min) * ratio)
        y = top + plot_height - (ratio * plot_height)
        parts.append(
            f'<line x1="{left:.1f}" y1="{y:.1f}" x2="{left + plot_width:.1f}" y2="{y:.1f}" stroke="#1b2a40" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{left - 10:.1f}" y="{y + 4:.1f}" text-anchor="end" font-size="11" fill="#93a4bf">{y_value:+.2f}</text>'
        )

    for index in range(x_ticks + 1):
        ratio = index / x_ticks
        x_value = x_min + ((x_max - x_min) * ratio)
        x = left + (ratio * plot_width)
        parts.append(
            f'<line x1="{x:.1f}" y1="{top:.1f}" x2="{x:.1f}" y2="{top + plot_height:.1f}" stroke="#132136" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x:.1f}" y="{top + plot_height + 18:.1f}" text-anchor="middle" font-size="11" fill="#93a4bf">{html.escape(fmt_hms_from_unix_seconds(x_value))}</text>'
        )

    zero_y = None
    if y_min <= 0.0 <= y_max:
        _, zero_y = scale_point(0.0, 0.0, 0.0, 1.0, y_min, y_max, 0.0, top, 1.0, plot_height)
        parts.append(
            f'<line x1="{left:.1f}" y1="{zero_y:.1f}" x2="{left + plot_width:.1f}" y2="{zero_y:.1f}" stroke="#5a6c8c" stroke-width="1.2" stroke-dasharray="4 4"/>'
        )

    legend_x = left + 12
    legend_y = top + 18
    for index, (name, values, color) in enumerate(series):
        polyline = polyline_points(xs, values, x_min, x_max, y_min, y_max, left, top, plot_width, plot_height)
        parts.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2.4" stroke-linejoin="round" stroke-linecap="round" points="{polyline}"/>'
        )
        scatter_points = scatter_points_svg(
            xs, values, color, x_min, x_max, y_min, y_max, left, top, plot_width, plot_height
        )
        if scatter_points:
            parts.append(scatter_points)
        fit_points = fit_line_points(xs, values, x_min, x_max, y_min, y_max, left, top, plot_width, plot_height)
        if fit_points:
            parts.append(
                f'<polyline fill="none" stroke="{color}" stroke-opacity="0.55" stroke-width="1.4" stroke-dasharray="6 5" points="{fit_points}"/>'
            )
        legend_item_y = legend_y + (index * 18)
        parts.append(
            f'<line x1="{legend_x:.1f}" y1="{legend_item_y:.1f}" x2="{legend_x + 18:.1f}" y2="{legend_item_y:.1f}" stroke="{color}" stroke-width="3"/>'
        )
        parts.append(
            f'<text x="{legend_x + 24:.1f}" y="{legend_item_y + 4:.1f}" font-size="11" fill="#c9d6ea">{html.escape(name)}</text>'
        )

    parts.append(
        f'<text x="{left + plot_width / 2:.1f}" y="{top + plot_height + 38:.1f}" text-anchor="middle" font-size="12" fill="#93a4bf">{html.escape(x_label)}</text>'
    )
    parts.append(
        f'<text x="18" y="{top + plot_height / 2:.1f}" text-anchor="middle" font-size="12" fill="#93a4bf" transform="rotate(-90 18 {top + plot_height / 2:.1f})">{html.escape(y_label)}</text>'
    )
    return "\n".join(parts)


def write_svg(rows: list[DriftRow], output_path: Path) -> None:
    epoch_seconds = [row.timestamp_unix_ms / 1000.0 for row in rows]
    pan_error = [row.pan_tracking_error_deg for row in rows]
    tilt_error = [row.tilt_tracking_error_deg for row in rows]
    pan_corr = [row.pan_correction_deg for row in rows]
    tilt_corr = [row.tilt_correction_deg for row in rows]

    header = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" width="1020" height="760" viewBox="0 0 1020 760" role="img">\n'
        '<rect width="1020" height="760" fill="#0f1726"/>\n'
        '<text x="28" y="34" font-size="22" font-weight="700" fill="#eef4ff">Heliostat Drift Analysis</text>\n'
        f'<text x="28" y="56" font-size="12" fill="#93a4bf">{html.escape(output_path.stem)}</text>\n'
    )
    chart1 = chart_svg(
        "Model residual vs time",
        "Time (UTC, HH:MM:SS)",
        "Residual (degrees)",
        epoch_seconds,
        [
            ("Pan residual", pan_error, "#60a5fa"),
            ("Tilt residual", tilt_error, "#f97316"),
        ],
        top_y=86,
    )
    chart2 = chart_svg(
        "Manual correction vs time",
        "Time (UTC, HH:MM:SS)",
        "Correction (degrees)",
        epoch_seconds,
        [
            ("Pan correction", pan_corr, "#34d399"),
            ("Tilt correction", tilt_corr, "#f472b6"),
        ],
        top_y=430,
    )
    footer = "\n</svg>\n"
    output_path.write_text(header + chart1 + "\n" + chart2 + footer, encoding="utf-8")


def print_summary(rows: list[DriftRow]) -> None:
    print(build_summary_text(rows))


def build_summary_text(rows: list[DriftRow]) -> str:
    first = rows[0]
    last = rows[-1]
    duration_hours = (last.timestamp_unix_ms - first.timestamp_unix_ms) / 3_600_000.0
    baseline_unix_ms = first_baseline_unix_ms(rows)

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

    lines = [
        "Drift analysis",
        f"- First baseline arm: {fmt_ts(baseline_unix_ms)} ({baseline_unix_ms})",
        f"- Samples: {len(rows)}",
        f"- Time range: {fmt_ts(first.timestamp_unix_ms)} -> {fmt_ts(last.timestamp_unix_ms)}",
        f"- Duration: {duration_hours:.2f} h",
        (
            f"- Captured target stability: bearing span "
            f"{max(target_bearing_values) - min(target_bearing_values):.4f}°, elevation span "
            f"{max(target_elevation_values) - min(target_elevation_values):.4f}°"
        ),
        (
            f"- Servo following command: mean(actual-target) pan {mean(servo_pan_error):+.3f}°, "
            f"tilt {mean(servo_tilt_error):+.3f}°"
        ),
        (
            f"- Model residual: mean(predicted-actual) pan {mean(model_pan_error):+.3f}°, "
            f"tilt {mean(model_tilt_error):+.3f}°"
        ),
        (
            f"- Manual correction trend vs time: pan {pan_time_slope:+.4f}°/min (r={pan_time_r:+.3f}), "
            f"tilt {tilt_time_slope:+.4f}°/min (r={tilt_time_r:+.3f})"
        ),
        f"- Manual pan correction vs sun bearing: {pan_bearing_slope:+.4f}°/bearing° (r={pan_bearing_r:+.3f})",
        f"- Manual tilt correction vs sun elevation: {tilt_elev_slope:+.4f}°/elevation° (r={tilt_elev_r:+.3f})",
    ]
    return "\n".join(lines)


def write_report(rows: list[DriftRow], json_path: Path) -> Path:
    reports_dir = json_path.parent / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    report_path = reports_dir / f"{fmt_report_stamp(first_baseline_unix_ms(rows))}.txt"
    report_path.write_text(build_summary_text(rows) + "\n", encoding="utf-8")
    return report_path


def default_svg_path(rows: list[DriftRow], json_path: Path) -> Path:
    reports_dir = json_path.parent / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    return reports_dir / f"{fmt_report_stamp(first_baseline_unix_ms(rows))}.svg"


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
    report_path = write_report(rows, args.json_file)
    if args.table:
        print_table(rows)
    if args.svg is not None:
        output_path = Path(args.svg) if args.svg else default_svg_path(rows, args.json_file)
        write_svg(rows, output_path)
        print()
        print(f"SVG written to: {output_path}")
    print()
    print(f"Report written to: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
