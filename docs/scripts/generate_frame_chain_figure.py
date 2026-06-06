#!/usr/bin/env python3
"""Generate the ORB-SLAM3 / ROS frame-chain figure used in the README."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


def add_box(ax, xy, wh, title, body, facecolor, edgecolor):
    x, y = xy
    w, h = wh
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.02,rounding_size=0.018",
        linewidth=1.6,
        facecolor=facecolor,
        edgecolor=edgecolor,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2.0, y + h * 0.66, title, ha="center", va="center", fontsize=13, weight="bold")
    ax.text(x + w / 2.0, y + h * 0.28, body, ha="center", va="center", fontsize=10.5)
    return patch


def add_arrow(ax, start, end, label, label_y_offset=0.03):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=16,
        linewidth=1.7,
        color="#374151",
        connectionstyle="arc3,rad=0.0",
    )
    ax.add_patch(arrow)
    mid_x = (start[0] + end[0]) / 2.0
    mid_y = (start[1] + end[1]) / 2.0 + label_y_offset
    ax.text(mid_x, mid_y, label, ha="center", va="bottom", fontsize=10, color="#374151")


def add_panel(ax, x, y, w, h, title, lines, accent):
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.02,rounding_size=0.018",
        linewidth=1.2,
        facecolor="#f9fafb",
        edgecolor=accent,
    )
    ax.add_patch(patch)
    ax.text(x + 0.03, y + h - 0.05, title, ha="left", va="top", fontsize=12.5, weight="bold", color=accent)
    text = "\n".join(f"• {line}" for line in lines)
    ax.text(x + 0.03, y + h - 0.10, text, ha="left", va="top", fontsize=10.3, color="#111827", linespacing=1.45)
    return patch


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    default_output = repo_root / "docs" / "images" / "orbslam3_frame_chain.png"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=default_output, help="Output image path")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
        }
    )

    fig = plt.figure(figsize=(15, 8.5), dpi=180)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.text(
        0.5,
        0.955,
        "ORB-SLAM3 ROS frame chain and the missing base_link -> camera_link extrinsic",
        ha="center",
        va="top",
        fontsize=18,
        weight="bold",
        color="#111827",
    )
    ax.text(
        0.5,
        0.915,
        "ORB-SLAM3 tracks the camera in the optical frame; ROS body motion should be expressed in base_link.",
        ha="center",
        va="top",
        fontsize=11.5,
        color="#4b5563",
    )

    y = 0.68
    w = 0.17
    h = 0.12
    xs = [0.05, 0.30, 0.55, 0.80]

    add_box(
        ax,
        (xs[0], y),
        (w, h),
        "map_frame",
        "world reference\nmap points live here",
        "#eff6ff",
        "#2563eb",
    )
    add_box(
        ax,
        (xs[1], y),
        (w, h),
        "base_link",
        "ROS body frame\nx forward, y left, z up",
        "#ecfeff",
        "#0891b2",
    )
    add_box(
        ax,
        (xs[2], y),
        (w, h),
        "camera_link",
        "mount frame\nextrinsic comes from URDF",
        "#f0fdf4",
        "#16a34a",
    )
    add_box(
        ax,
        (xs[3], y),
        (w, h),
        "camera_optical_frame",
        "optical frame\nz forward, x right, y down",
        "#fff7ed",
        "#ea580c",
    )

    add_arrow(ax, (xs[0] + w, y + h / 2.0), (xs[1], y + h / 2.0), "publish TF")
    add_arrow(ax, (xs[1] + w, y + h / 2.0), (xs[2], y + h / 2.0), "URDF / robot_state_publisher")
    add_arrow(ax, (xs[2] + w, y + h / 2.0), (xs[3], y + h / 2.0), "REP-103 optical rotation")

    ax.text(
        0.5,
        0.575,
        r"Observed pose:  $T^{world}_{base} = T^{world}_{camera} \cdot T^{camera}_{base}$",
        ha="center",
        va="center",
        fontsize=13,
        color="#111827",
    )
    ax.text(
        0.5,
        0.54,
        r"ORB-SLAM3 returns $T_{cw}$  ->  wrapper inverts to camera pose and then applies the base_link transform.",
        ha="center",
        va="center",
        fontsize=11.2,
        color="#4b5563",
    )

    add_panel(
        ax,
        0.06,
        0.16,
        0.40,
        0.28,
        "What the frames mean",
        [
            "base_link is the ROS body frame, so forward motion is +x.",
            "camera_optical_frame follows the camera convention, so forward motion is +z.",
            "map_points are published in map_frame, not in the camera frame.",
        ],
        "#2563eb",
    )
    add_panel(
        ax,
        0.54,
        0.16,
        0.40,
        0.28,
        "Why z grows when the camera moves forward",
        [
            "The images are fed to ORB-SLAM3 in the optical frame.",
            "If base_link -> camera_link is missing, the wrapper falls back to identity.",
            "The visualization then stays aligned with the camera axis, so forward motion appears on +z.",
        ],
        "#ea580c",
    )

    ax.text(
        0.06,
        0.08,
        "Minimal test setup: define base_link -> camera_link in URDF, and keep camera_optical_frame as the REP-103 optical frame.",
        ha="left",
        va="center",
        fontsize=10.8,
        color="#374151",
    )

    fig.savefig(args.output, bbox_inches="tight")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
