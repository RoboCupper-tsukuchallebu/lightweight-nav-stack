#!/usr/bin/env python3
"""Top-down PNG of a saved map (map.pcd + trajectory.csv) with a metric grid, plus its basic size numbers.

    render_map.py <map_directory> [--output map.png] [--bin-m 0.05] [--json]

Prints the bounding box size of the map points and of the trajectory, the path length (xy) and the
start-end distance, so the shape can be compared with the place the robot actually drove.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

MINOR_GRID_M = 1.0
MAJOR_GRID_M = 5.0
FIGURE_LONG_SIDE_INCHES = 14.0
FIGURE_MARGIN_M = 1.0
MAXIMUM_DENSITY_BINS = 6000  # per axis; a larger map is rendered coarser instead of running out of memory


def read_pcd_points(pcd_path):
    """Reads the x y z float32 PCD that localization_node writes (binary or ascii). Returns an (N, 3) array."""
    raw = Path(pcd_path).read_bytes()
    header = {}
    offset = 0
    while True:
        line_end = raw.index(b"\n", offset)
        line = raw[offset:line_end].decode("ascii").strip()
        offset = line_end + 1
        if not line or line.startswith("#"):
            continue
        key, _, rest = line.partition(" ")
        header[key] = rest.strip()
        if key == "DATA":
            break
    point_count = int(header["POINTS"])
    if header["FIELDS"].split()[:3] != ["x", "y", "z"]:
        raise ValueError(f"{pcd_path}: FIELDS must start with x y z, got '{header['FIELDS']}'")
    if point_count == 0:
        return np.zeros((0, 3), dtype=np.float32)
    field_count = len(header["FIELDS"].split())
    if header["DATA"] == "binary":
        points = np.frombuffer(raw, dtype="<f4", count=point_count * field_count, offset=offset)
        return points.reshape(point_count, field_count)[:, :3].copy()
    if header["DATA"] == "ascii":
        values = np.loadtxt(raw[offset:].decode("ascii").splitlines(), dtype=np.float32, ndmin=2)
        return values[:, :3].copy()
    raise ValueError(f"{pcd_path}: unsupported DATA kind '{header['DATA']}'")


def read_trajectory(csv_path):
    """Reads trajectory.csv. Returns an (M, 2) array of x, y (m)."""
    with Path(csv_path).open(newline="") as csv_file:
        rows = [(float(row["x"]), float(row["y"])) for row in csv.DictReader(csv_file)]
    return np.array(rows, dtype=np.float64).reshape(-1, 2)


def extent(values_xy):
    """(width_x, width_y) of the bounding box, (0, 0) for an empty array."""
    if len(values_xy) == 0:
        return (0.0, 0.0)
    spans = values_xy.max(axis=0) - values_xy.min(axis=0)
    return (float(spans[0]), float(spans[1]))


def path_length(trajectory_xy):
    if len(trajectory_xy) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(trajectory_xy, axis=0), axis=1).sum())


def start_end_distance(trajectory_xy):
    if len(trajectory_xy) < 2:
        return 0.0
    return float(np.linalg.norm(trajectory_xy[-1] - trajectory_xy[0]))


def compute_statistics(points, trajectory_xy):
    map_width_x, map_width_y = extent(points[:, :2])
    path_width_x, path_width_y = extent(trajectory_xy)
    height = float(points[:, 2].max() - points[:, 2].min()) if len(points) else 0.0
    return {
        "map_points": int(len(points)),
        "keyframes": int(len(trajectory_xy)),
        "map_bbox_x_m": map_width_x,
        "map_bbox_y_m": map_width_y,
        "map_bbox_z_m": height,
        "path_bbox_x_m": path_width_x,
        "path_bbox_y_m": path_width_y,
        "path_length_m": path_length(trajectory_xy),
        "start_end_distance_m": start_end_distance(trajectory_xy),
    }


def render_png(points, trajectory_xy, output_path, bin_m, title):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MultipleLocator

    all_xy = np.vstack([points[:, :2].astype(np.float64), trajectory_xy]) if len(points) else trajectory_xy
    if len(all_xy) == 0:
        raise ValueError("nothing to draw: the map and the trajectory are both empty")
    low = all_xy.min(axis=0) - FIGURE_MARGIN_M
    high = all_xy.max(axis=0) + FIGURE_MARGIN_M
    width_m, height_m = high - low
    scale = FIGURE_LONG_SIDE_INCHES / max(width_m, height_m)
    figure, axes = plt.subplots(figsize=(width_m * scale + 1.5, height_m * scale + 1.5), dpi=120)

    if len(points):
        bin_size = max(bin_m, max(width_m, height_m) / MAXIMUM_DENSITY_BINS)
        edges_x = np.arange(low[0], high[0] + bin_size, bin_size)
        edges_y = np.arange(low[1], high[1] + bin_size, bin_size)
        counts, _, _ = np.histogram2d(points[:, 0], points[:, 1], bins=[edges_x, edges_y])
        axes.imshow(np.log1p(counts).T, origin="lower", extent=(low[0], edges_x[-1], low[1], edges_y[-1]),
                    cmap="Greys", interpolation="nearest", zorder=1)
    if len(trajectory_xy):
        axes.plot(trajectory_xy[:, 0], trajectory_xy[:, 1], "-", color="tab:red", linewidth=1.2, zorder=3, label="trajectory")
        axes.plot(*trajectory_xy[0], "o", color="tab:green", markersize=8, zorder=4, label="start")
        axes.plot(*trajectory_xy[-1], "s", color="tab:blue", markersize=8, zorder=4, label="end")
    axes.set_xlim(low[0], high[0])
    axes.set_ylim(low[1], high[1])
    axes.set_aspect("equal")
    axes.xaxis.set_minor_locator(MultipleLocator(MINOR_GRID_M))
    axes.yaxis.set_minor_locator(MultipleLocator(MINOR_GRID_M))
    axes.xaxis.set_major_locator(MultipleLocator(MAJOR_GRID_M))
    axes.yaxis.set_major_locator(MultipleLocator(MAJOR_GRID_M))
    axes.grid(which="minor", color="0.85", linewidth=0.5, zorder=0)
    axes.grid(which="major", color="0.55", linewidth=0.9, zorder=0)
    axes.set_xlabel("map x [m]")
    axes.set_ylabel("map y [m]")
    axes.set_title(title)
    axes.legend(loc="upper right")
    figure.tight_layout()
    figure.savefig(output_path)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_directory", type=Path, help="directory with map.pcd and trajectory.csv")
    parser.add_argument("--output", type=Path, help="PNG path (default: <map_directory>/map_topdown.png)")
    parser.add_argument("--bin-m", type=float, default=0.05, help="density image cell size in metres")
    parser.add_argument("--json", action="store_true", help="print the numbers as one JSON object")
    arguments = parser.parse_args(argv)

    points = read_pcd_points(arguments.map_directory / "map.pcd")
    trajectory_xy = read_trajectory(arguments.map_directory / "trajectory.csv")
    statistics = compute_statistics(points, trajectory_xy)
    output_path = arguments.output or arguments.map_directory / "map_topdown.png"
    render_png(points, trajectory_xy, output_path, arguments.bin_m,
               title=f"{arguments.map_directory.name}: {statistics['map_points']} points, "
                     f"{statistics['keyframes']} keyframes")

    if arguments.json:
        print(json.dumps(statistics, sort_keys=True))
    else:
        print(f"png: {output_path}")
        for name, value in statistics.items():
            print(f"{name}: {value:.3f}" if isinstance(value, float) else f"{name}: {value}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
