"""render_map.py: the PCD / trajectory readers and the size numbers (no plotting needed)."""

import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_map  # noqa: E402

PCD_HEADER = (
    "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
    "WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA {kind}\n"
)


def write_binary_pcd(path, points):
    body = b"".join(struct.pack("<fff", *point) for point in points)
    path.write_bytes(PCD_HEADER.format(count=len(points), kind="binary").encode("ascii") + body)


def write_trajectory(path, positions):
    lines = ["stamp_sec,x,y,z,roll,pitch,yaw,position_std_m"]
    for index, (x, y) in enumerate(positions):
        lines.append(f"{index},{x},{y},0,0,0,0,0.1")
    path.write_text("\n".join(lines) + "\n")


def test_reads_a_binary_pcd(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    write_binary_pcd(pcd_path, [(1.0, 2.0, 3.0), (-4.0, 5.0, 0.5)])

    points = render_map.read_pcd_points(pcd_path)

    assert points.shape == (2, 3)
    assert points[1].tolist() == [-4.0, 5.0, 0.5]


def test_reads_an_ascii_pcd(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    pcd_path.write_text(PCD_HEADER.format(count=2, kind="ascii") + "1 2 3\n4 5 6\n")

    points = render_map.read_pcd_points(pcd_path)

    assert points.tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]


def test_reads_an_empty_pcd(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    write_binary_pcd(pcd_path, [])

    assert render_map.read_pcd_points(pcd_path).shape == (0, 3)


def test_rejects_a_pcd_without_xyz_fields(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    pcd_path.write_text(PCD_HEADER.replace("FIELDS x y z", "FIELDS a b c").format(count=0, kind="ascii"))

    with pytest.raises(ValueError):
        render_map.read_pcd_points(pcd_path)


def test_statistics_of_an_l_shaped_drive(tmp_path):
    # Arrange: 10 m along +x, then 3 m along +y (13 m of path), map points on a 10 m x 3 m rectangle.
    write_trajectory(tmp_path / "trajectory.csv", [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 3.0)])
    trajectory_xy = render_map.read_trajectory(tmp_path / "trajectory.csv")
    points = np.array([[0.0, 0.0, 0.0], [10.0, 3.0, 1.5], [5.0, 1.0, 0.2]], dtype=np.float32)

    # Act
    statistics = render_map.compute_statistics(points, trajectory_xy)

    # Assert
    assert statistics["map_points"] == 3
    assert statistics["keyframes"] == 4
    assert statistics["map_bbox_x_m"] == pytest.approx(10.0)
    assert statistics["map_bbox_y_m"] == pytest.approx(3.0)
    assert statistics["map_bbox_z_m"] == pytest.approx(1.5)
    assert statistics["path_length_m"] == pytest.approx(13.0)
    assert statistics["start_end_distance_m"] == pytest.approx(math.hypot(10.0, 3.0))


def test_a_closed_loop_has_zero_start_end_distance():
    loop_xy = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0], [0.0, 0.0]])

    assert render_map.path_length(loop_xy) == pytest.approx(16.0)
    assert render_map.start_end_distance(loop_xy) == pytest.approx(0.0)


def test_statistics_of_nothing_are_zero():
    statistics = render_map.compute_statistics(np.zeros((0, 3), dtype=np.float32), np.zeros((0, 2)))

    assert statistics["map_bbox_x_m"] == 0.0
    assert statistics["path_length_m"] == 0.0
    assert statistics["start_end_distance_m"] == 0.0
