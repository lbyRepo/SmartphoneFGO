"""Tiny SVG plotting helpers that avoid optional plotting dependencies."""

from __future__ import annotations

from pathlib import Path

from python_tools.evaluation.estimators import Estimate
from python_tools.evaluation.math_utils import Vec3, distance
from python_tools.evaluation.synthetic import SyntheticDataset


COLORS = {
    "truth": "#111111",
    "android_fused": "#8A63D2",
    "wls": "#1F6FEB",
    "ekf": "#2DA44E",
    "fgo": "#D1242F",
}


def _scale_points(points: list[tuple[float, float]], width: int, height: int, pad: int) -> list[tuple[float, float]]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)
    scaled = []
    for x, y in points:
        sx = pad + (x - min_x) / span_x * (width - 2 * pad)
        sy = height - pad - (y - min_y) / span_y * (height - 2 * pad)
        scaled.append((sx, sy))
    return scaled


def _polyline(points: list[tuple[float, float]], color: str, width: float = 2.0) -> str:
    data = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline points="{data}" fill="none" stroke="{color}" stroke-width="{width}" />'


def write_trajectory_svg(path: Path, runs: dict[str, tuple[Estimate, ...]], dataset: SyntheticDataset) -> None:
    width, height, pad = 900, 600, 60
    truth_points = [(epoch.truth.position_ecef_m[0], epoch.truth.position_ecef_m[1]) for epoch in dataset.epochs]
    fused_points = [(point[0], point[1]) for point in dataset.fused_positions_ecef_m]
    all_points = truth_points + fused_points
    for estimates in runs.values():
        all_points.extend((estimate.position_ecef_m[0], estimate.position_ecef_m[1]) for estimate in estimates)
    scaled_all = _scale_points(all_points, width, height, pad)
    cursor = 0

    def take(count: int) -> list[tuple[float, float]]:
        nonlocal cursor
        points = scaled_all[cursor : cursor + count]
        cursor += count
        return points

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="24" y="32" font-size="20" font-family="sans-serif">Synthetic trajectory XY comparison</text>',
    ]
    parts.append(_polyline(take(len(truth_points)), COLORS["truth"], 3.0))
    parts.append(_polyline(take(len(fused_points)), COLORS["android_fused"], 2.0))
    for name, estimates in runs.items():
        parts.append(_polyline(take(len(estimates)), COLORS.get(name, "#555555"), 2.0))
    legend_y = 62
    for name in ["truth", "android_fused", *runs.keys()]:
        color = COLORS.get(name, "#555555")
        parts.append(f'<line x1="24" y1="{legend_y}" x2="52" y2="{legend_y}" stroke="{color}" stroke-width="3" />')
        parts.append(f'<text x="60" y="{legend_y + 5}" font-size="14" font-family="sans-serif">{name}</text>')
        legend_y += 22
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def write_error_svg(path: Path, runs: dict[str, tuple[Estimate, ...]], dataset: SyntheticDataset) -> None:
    width, height, pad = 900, 500, 60
    series: dict[str, list[tuple[float, float]]] = {}
    for name, estimates in runs.items():
        series[name] = [
            (
                estimate.time_seconds,
                distance(estimate.position_ecef_m, dataset.epochs[estimate.epoch_index].truth.position_ecef_m),
            )
            for estimate in estimates
        ]
    all_points = [point for points in series.values() for point in points]
    scaled_all = _scale_points(all_points, width, height, pad)
    cursor = 0
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="24" y="32" font-size="20" font-family="sans-serif">Position error over time</text>',
    ]
    for name, points in series.items():
        scaled = scaled_all[cursor : cursor + len(points)]
        cursor += len(points)
        parts.append(_polyline(scaled, COLORS.get(name, "#555555"), 2.0))
    legend_y = 62
    for name in runs:
        color = COLORS.get(name, "#555555")
        parts.append(f'<line x1="24" y1="{legend_y}" x2="52" y2="{legend_y}" stroke="{color}" stroke-width="3" />')
        parts.append(f'<text x="60" y="{legend_y + 5}" font-size="14" font-family="sans-serif">{name}</text>')
        legend_y += 22
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")
