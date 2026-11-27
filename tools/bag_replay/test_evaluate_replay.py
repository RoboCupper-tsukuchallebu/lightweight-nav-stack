"""evaluate_replay.py: the seven rules of Issue #93 section 4.5, checked against numbers worked out by hand on made-up data."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import evaluate_replay  # noqa: E402

REFERENCE_LATITUDE_DEG = 10.0  # made-up place
REFERENCE_LONGITUDE_DEG = 20.0
METERS_PER_DEGREE = 6378137.0 * math.pi / 180.0


def plane_to_geodetic(east_m, north_m):
    latitude = REFERENCE_LATITUDE_DEG + north_m / METERS_PER_DEGREE
    longitude = REFERENCE_LONGITUDE_DEG + east_m / (METERS_PER_DEGREE * math.cos(math.radians(REFERENCE_LATITUDE_DEG)))
    return latitude, longitude


def make_gnss(times_s, plane_points):
    latitudes, longitudes = zip(*[plane_to_geodetic(east, north) for east, north in plane_points])
    return {"times_s": np.array(times_s, dtype=float), "latitudes_deg": np.array(latitudes), "longitudes_deg": np.array(longitudes)}


def make_poses(times_s, xs, ys=None, zs=None, yaws=None):
    count = len(times_s)
    columns = [xs, ys if ys is not None else [0.0] * count, zs if zs is not None else [0.0] * count,
               yaws if yaws is not None else [0.0] * count]
    return {"times_s": np.array(times_s, dtype=float), "poses": np.column_stack(columns).astype(float)}


def test_plane_uses_the_equatorial_radius_and_the_cosine_of_the_reference_latitude():
    plane = evaluate_replay.geodetic_to_plane([10.001], [20.002], 10.0, 20.0)

    assert plane[0, 0] == pytest.approx(219.2566, abs=1e-3)  # 0.002 deg * 111319.49 m/deg * cos(10 deg)
    assert plane[0, 1] == pytest.approx(111.3195, abs=1e-3)  # 0.001 deg * 111319.49 m/deg


def test_nearest_pairs_drop_gnss_points_farther_than_one_second():
    source_times = [0.0, 5.0, 10.0]
    target_times = [0.4, 4.0, 10.9]  # gaps 0.4 / 1.0 / 0.9: the 5.0 s row is exactly 1.0 s away (still used)

    sources, targets = evaluate_replay.nearest_time_pairs(source_times, target_times, 1.0)

    assert list(sources) == [0, 1, 2]
    assert list(targets) == [0, 1, 2]
    sources, _ = evaluate_replay.nearest_time_pairs([5.0 + 0.01], target_times, 1.0)
    assert list(sources) == []


def test_rigid_fit_recovers_a_rotation_and_translation_without_residual():
    points = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]])
    angle = math.radians(30.0)
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    moved = points @ rotation.T + np.array([100.0, -50.0])

    statistics = evaluate_replay.aligned_statistics(points, moved)

    assert statistics["rms_m"] == pytest.approx(0.0, abs=1e-9)
    assert statistics["max_m"] == pytest.approx(0.0, abs=1e-9)


def test_rigid_fit_does_not_scale():
    # x = 0..3 against east = 0, 2, 4, 6. Centred: -1.5..1.5 against -3..3, so the remaining gaps are 1.5, 0.5, -0.5, -1.5.
    moving = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    fixed = np.array([[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0]])

    statistics = evaluate_replay.aligned_statistics(moving, fixed)

    assert statistics["rms_m"] == pytest.approx(math.sqrt(1.25))
    assert statistics["max_m"] == pytest.approx(1.5)


def test_rigid_fit_never_reflects():
    moving = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]])
    mirrored = moving * np.array([1.0, -1.0])

    statistics = evaluate_replay.aligned_statistics(moving, mirrored)

    assert statistics["max_m"] > 1.0


def test_interpolation_is_linear_and_turns_yaw_the_short_way():
    sample_times = [0.0, 0.2]
    sample_poses = np.array([[0.0, 0.0, 0.0, math.radians(179.0)], [2.0, 4.0, 1.0, math.radians(-179.0)]])

    poses, is_valid = evaluate_replay.interpolate_poses(sample_times, sample_poses, [0.1])

    assert is_valid[0]
    assert poses[0, :3] == pytest.approx([1.0, 2.0, 0.5])
    assert abs(abs(poses[0, 3]) - math.pi) < 1e-9


def test_interpolation_needs_samples_within_0_2_s_on_both_sides():
    sample_times = [0.0, 0.5, 1.0]
    sample_poses = np.zeros((3, 4))

    _, is_valid = evaluate_replay.interpolate_poses(sample_times, sample_poses, [0.0, 0.25, 0.5, -0.1, 1.1])

    # 0.0 and 0.5 sit on a sample; 0.25 has 0.25 s to each side; -0.1 and 1.1 have no sample on one side.
    assert list(is_valid) == [True, False, True, False, False]


def test_mapping_numbers_of_a_line_with_a_gnss_offset():
    times = [0.0, 1.0, 2.0, 3.0]
    trajectory = make_poses(times, [0.0, 1.0, 2.0, 3.0], zs=[0.0, 0.5, -0.25, 0.0], yaws=[0.0, 0.0, 0.0, math.radians(90.0)])
    # North offsets +0.1, -0.1, -0.1, +0.1 are balanced (no rotation, no shift): every point stays 0.1 m away after the fit.
    # The GNSS is also rotated 90 deg and moved; the fit has to undo that.
    plane = [(5.0 - north, 7.0 + east) for east, north in zip([0.0, 1.0, 2.0, 3.0], [0.1, -0.1, -0.1, 0.1])]
    gnss = make_gnss([0.2, 1.1, 2.0, 3.5], plane)

    result = evaluate_replay.evaluate_mapping(trajectory, gnss)

    assert result["start_end_distance_m"] == pytest.approx(3.0)
    assert result["start_end_yaw_difference_rad"] == pytest.approx(math.radians(90.0))
    assert result["height_range_m"] == pytest.approx(0.75)
    assert result["gnss_aligned"]["count"] == 4
    assert result["gnss_aligned"]["rms_m"] == pytest.approx(0.1, abs=1e-6)
    assert result["gnss_aligned"]["max_m"] == pytest.approx(0.1, abs=1e-6)


def test_mapping_skips_trajectory_rows_without_a_gnss_point_within_one_second():
    trajectory = make_poses([0.0, 1.0, 2.0, 10.0], [0.0, 1.0, 2.0, 50.0])
    gnss = make_gnss([0.0, 1.0, 2.0], [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)])

    result = evaluate_replay.evaluate_mapping(trajectory, gnss)

    assert result["gnss_aligned"]["count"] == 3
    assert result["gnss_aligned"]["rms_m"] == pytest.approx(0.0, abs=1e-6)


def test_state_fractions_exclude_initializing_and_stale_samples_from_the_denominators():
    states = ["INITIALIZING"] * 2 + ["GPS_ONLY"] * 3 + ["INPUT_STALE"] * 2 + ["GPS_ONLY"] * 1 + ["ODOM_ONLY"] * 2

    result = evaluate_replay.state_fractions(states)

    assert result["input_stale_fraction"] == pytest.approx(2 / 8)  # 2 / (10 - 2)
    assert result["gps_only_fraction"] == pytest.approx(4 / 6)  # 4 / (10 - 2 - 2)
    assert result["tracking_fraction"] is None


def test_tracking_fraction_counts_from_the_first_tracking_without_stale_samples():
    states = ["INITIALIZING", "ODOM_ONLY", "TRACKING", "TRACKING", "INPUT_STALE", "LOST", "TRACKING"]

    result = evaluate_replay.state_fractions(states)

    assert result["tracking_fraction"] == pytest.approx(3 / 4)  # span of 5 from the first TRACKING, 1 stale -> 3 / 4


def test_fractions_are_none_when_the_denominator_is_empty():
    result = evaluate_replay.state_fractions(["INITIALIZING"])

    assert result["input_stale_fraction"] is None
    assert result["gps_only_fraction"] is None


def test_localization_compares_against_map_rows_after_the_first_tracking_only():
    row_times = [10.0, 11.0, 12.0, 13.0]
    map_trajectory = make_poses(row_times, [0.0, 1.0, 2.0, 3.0], zs=[0.0] * 4)
    online_times = [9.9, 10.1, 10.9, 11.1, 11.9, 12.1, 12.95, 13.05]
    online_x = [t - 10.0 + 0.5 for t in online_times]  # 0.5 m ahead of the map
    online_z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
    online_path = make_poses(online_times, online_x, zs=online_z)

    result = evaluate_replay.evaluate_localization(online_path, map_trajectory, None, None, first_tracking_stamp=11.0)

    against_map = result["against_map_trajectory"]
    assert against_map["count"] == 3  # rows at 11, 12, 13 (the row at 10 is before the first TRACKING)
    assert against_map["rms_m"] == pytest.approx(0.5)
    assert against_map["max_m"] == pytest.approx(0.5)
    assert against_map["height_difference_max_m"] == pytest.approx(0.15)  # row at 13: halfway between 0.0 and 0.3


def test_localization_gnss_comparison_aligned_and_not_aligned():
    map_trajectory = make_poses([0.0, 10.0], [0.0, 10.0])
    online_times = [float(second) for second in range(0, 11)]
    online_path = make_poses(online_times, [float(second) + 1.0 for second in online_times])  # 1 m east of the map
    gnss_times = [2.0, 5.0, 8.0]
    gnss = make_gnss(gnss_times, [(2.0, 0.0), (5.0, 0.0), (8.0, 0.0)])
    origin = plane_to_geodetic(0.0, 0.0)

    result = evaluate_replay.evaluate_localization(online_path, map_trajectory, gnss, origin, first_tracking_stamp=3.0)

    # Only the points after the first TRACKING (5 s, 8 s) are used. The first GNSS point (the plane reference) is 2 m east
    # of the map origin, so the not-aligned comparison is against east of the origin: online x = 6, 9 against 5, 8.
    assert result["against_gnss_aligned"]["count"] == 2
    assert result["against_gnss_aligned"]["rms_m"] == pytest.approx(0.0, abs=1e-6)
    assert result["against_gnss_not_aligned"]["count"] == 2
    assert result["against_gnss_not_aligned"]["rms_m"] == pytest.approx(1.0, abs=1e-6)


def test_localization_without_a_tracking_state_compares_nothing():
    result = evaluate_replay.evaluate_localization(make_poses([0.0], [0.0]), make_poses([0.0], [0.0]), None, None, None)

    assert result == {"first_tracking_time_s": None}


def test_save_timings_come_from_the_map_save_lines_of_each_save():
    log = "\n".join([
        "[INFO] [1.0] [localization_node]: unrelated line",
        "[INFO] [2.0] [localization_node]: map save: preliminary map written 1.5 s after the start of the save",
        "[INFO] [3.0] [localization_node]: map save: optimized map written 40.2 s after the start of the save",
        "[INFO] [4.0] [localization_node]: map save: final map written 191.2 s after the start of the save",
        "[INFO] [9.0] [localization_node]: map save: preliminary map written 2.0 s after the start of the save",
    ])

    saves = evaluate_replay.parse_save_timings(log)

    assert saves == [{"preliminary": 1.5, "optimized": 40.2, "final": 191.2}, {"preliminary": 2.0}]


def test_a_save_that_starts_at_a_later_stage_is_counted_as_a_second_save():
    # save_map writes all three stages; the save at the stop starts at 'optimized' (no new keyframe since).
    log = "\n".join([
        "map save: preliminary map written 1.0 s after the start of the save",
        "map save: optimized map written 30.0 s after the start of the save",
        "map save: final map written 190.0 s after the start of the save",
        "map save: optimized map written 25.0 s after the start of the save",
        "map save: final map written 180.0 s after the start of the save",
    ])

    saves = evaluate_replay.parse_save_timings(log)

    assert saves == [{"preliminary": 1.0, "optimized": 30.0, "final": 190.0}, {"optimized": 25.0, "final": 180.0}]


def test_a_save_that_stops_midway_is_followed_by_a_second_save_from_the_start():
    log = "\n".join([
        "map save: preliminary map written 1.0 s after the start of the save",
        "map save: optimized map written 30.0 s after the start of the save",
        "map save: preliminary map written 2.0 s after the start of the save",
    ])

    assert evaluate_replay.parse_save_timings(log) == [{"preliminary": 1.0, "optimized": 30.0}, {"preliminary": 2.0}]


def test_map_gps_origin_is_none_without_an_origin(tmp_path):
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text("has_gps_origin: false\n")
    assert evaluate_replay.read_map_gps_origin(yaml_path) is None
    yaml_path.write_text("has_gps_origin: true\norigin_lat: 10.5\norigin_lon: 20.25\n")
    assert evaluate_replay.read_map_gps_origin(yaml_path) == (10.5, 20.25)


def test_a_replay_directory_is_evaluated_end_to_end(tmp_path):
    (tmp_path / "map").mkdir()
    (tmp_path / "map" / "trajectory.csv").write_text(
        "stamp_sec,x,y,z,roll,pitch,yaw,position_std_m\n0,0,0,0,0,0,0,nan\n1,3,4,1,0,0,0.5,nan\n")
    (tmp_path / "status.csv").write_text(
        "stamp_sec,state,scan_ms,match_ms,lidar_odom_inlier_ratio,odom_only_distance_m,last_error\n"
        "0,INITIALIZING,0,0,0,0,\n1,GPS_ONLY,1,1,1,0,\n")
    latitude, longitude = plane_to_geodetic(0.0, 0.0)
    (tmp_path / "gnss.csv").write_text(
        "bag_time_ns,latitude,longitude,altitude,status,cov_east,cov_north,cov_up\n"
        f"0,{latitude},{longitude},0,2,0,0,0\n")
    (tmp_path / "node.log").write_text("map save: final map written 3.5 s after the start of the save\n")

    result = evaluate_replay.evaluate_directory(tmp_path, "mapping", None)

    assert result["start_end_distance_m"] == pytest.approx(5.0)
    assert result["gps_only_fraction"] == pytest.approx(1.0)
    assert result["save_timings_s"] == [{"final": 3.5}]
    assert result["gnss_aligned"]["count"] == 2  # both rows are within 1 s of the one GNSS point
