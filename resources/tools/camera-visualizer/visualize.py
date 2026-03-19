#!/usr/bin/env python3
"""Portable Transform3d camera visualizer.

Reads a JSON file that lists cameras with Transform3d-style poses and
renders them in a local 3D matplotlib window or saves a PNG.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class CameraPose:
    name: str
    position: np.ndarray
    rotation_rpy: np.ndarray


def _ensure_backend(output: Optional[str]) -> None:
    if output or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")


def _radians(value: float, degrees: bool) -> float:
    return math.radians(value) if degrees else value


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create rotation matrix from roll (x), pitch (y), yaw (z).

    Uses the WPILib convention: apply roll about X, then pitch about Y,
    then yaw about Z (intrinsic rotations), equivalent to Rz * Ry * Rx.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return rz @ ry @ rx


def load_cameras(path: Path) -> List[CameraPose]:
    data = json.loads(path.read_text())
    cameras = []
    for entry in data.get("cameras", []):
        name = entry.get("name", "Unnamed")
        t3d = entry.get("transform3d", {})
        degrees = bool(t3d.get("degrees", True))
        x = float(t3d.get("x", 0.0))
        y = float(t3d.get("y", 0.0))
        z = float(t3d.get("z", 0.0))
        roll = _radians(float(t3d.get("roll", 0.0)), degrees)
        pitch = _radians(float(t3d.get("pitch", 0.0)), degrees)
        yaw = _radians(float(t3d.get("yaw", 0.0)), degrees)
        cameras.append(
            CameraPose(
                name=name,
                position=np.array([x, y, z], dtype=float),
                rotation_rpy=np.array([roll, pitch, yaw], dtype=float),
            )
        )

    if not cameras:
        raise ValueError("No cameras found in JSON. Add entries under 'cameras'.")

    return cameras


def plot_axes(
    ax: plt.Axes,
    origin: np.ndarray,
    rotation: np.ndarray,
    length: float,
    label: Optional[str] = None,
) -> None:
    axes = np.eye(3) * length
    rotated = rotation @ axes
    colors = ["#d62728", "#2ca02c", "#1f77b4"]

    for i in range(3):
        vec = rotated[:, i]
        ax.plot(
            [origin[0], origin[0] + vec[0]],
            [origin[1], origin[1] + vec[1]],
            [origin[2], origin[2] + vec[2]],
            color=colors[i],
            linewidth=2,
        )

    if label:
        ax.text(origin[0], origin[1], origin[2], f" {label}", fontsize=10)


def plot_look_arrow(
    ax: plt.Axes,
    origin: np.ndarray,
    rotation: np.ndarray,
    length: float,
    color: str = "#ff7f0e",
) -> None:
    """Draw an arrow pointing in the camera's forward direction."""
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        forward[0],
        forward[1],
        forward[2],
        length=length,
        normalize=True,
        color=color,
        linewidth=2,
        arrow_length_ratio=0.25,
    )


def set_axes_equal(ax: plt.Axes) -> None:
    limits = np.array(
        [
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ]
    )
    centers = np.mean(limits, axis=1)
    radius = 0.5 * np.max(limits[:, 1] - limits[:, 0])
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize Transform3d camera poses in 3D."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path(__file__).with_name("cameras.json"),
        help="Path to cameras JSON file.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Optional output PNG path (headless mode).",
    )
    parser.add_argument(
        "--axes-length",
        type=float,
        default=0.2,
        help="Length of each axis in meters.",
    )
    parser.add_argument(
        "--no-robot",
        action="store_true",
        help="Disable drawing the robot origin axes.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _ensure_backend(args.output)

    cameras = load_cameras(args.input)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    if not args.no_robot:
        plot_axes(ax, np.zeros(3), np.eye(3), args.axes_length, "Robot")

    for camera in cameras:
        rotation = rotation_matrix(*camera.rotation_rpy)
        plot_axes(
            ax,
            camera.position,
            rotation,
            args.axes_length,
            camera.name,
        )
        plot_look_arrow(
            ax,
            camera.position,
            rotation,
            args.axes_length * 1.5,
        )

    positions = np.array([cam.position for cam in cameras])
    all_points = np.vstack([positions, np.zeros(3)])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Camera Transform3d Poses")
    ax.scatter(all_points[:, 0], all_points[:, 1], all_points[:, 2], s=30)

    set_axes_equal(ax)

    if args.output:
        plt.tight_layout()
        plt.savefig(args.output, dpi=160)
        print(f"Saved visualization to {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
