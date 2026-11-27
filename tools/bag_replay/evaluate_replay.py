#!/usr/bin/env python3
"""Numbers of one bag replay (the table of Issue #93 section 5.3), from the output directory of replay_session.py.

    evaluate_replay.py <replay output directory> [--mode mapping|localization] [--map-directory DIR] [--json-output FILE]

The rules of the calculation are fixed (Issue #93 section 4.5; change them only together with the acceptance criteria):

 1. Plane: the first GNSS point is the reference. east = longitude difference (rad) * cos(reference latitude) * 6378137 m,
    north = latitude difference (rad) * 6378137 m.
 2. Mapping trajectory: the saved map/trajectory.csv. Start-end gap = horizontal distance of the first and the last row.
 3. Pairing with GNSS: each trajectory row gets the GNSS point nearest in time; a time difference above 1 s is not used.
 4. Alignment: 2D rigid transform (rotation and translation, no scaling) by least squares; RMS and maximum after it.
 5. Localization trajectory: online_path.csv linearly interpolated at the time of the other side, only when an output lies
    within 0.2 s before and within 0.2 s after. Against the rows of the map trajectory: not aligned; rows before the
    first TRACKING are left out. Against the GNSS points (after the first TRACKING): aligned as in 4, and, when the map has
    a GNSS origin, also not aligned.
 6. State fractions (counted in status.csv samples): INPUT_STALE / (all - INITIALIZING);
    GPS_ONLY / (all - INITIALIZING - INPUT_STALE); TRACKING / (all from the first TRACKING - INPUT_STALE in that span).
 7. Save time: the seconds of the "map save: <stage> map written <s> s after the start of the save" lines of node.log.

gnss.csv holds latitudes and longitudes: it is not published (this script prints only differences in metres).
"""

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

EARTH_RADIUS_M = 6378137.0
MAXIMUM_GNSS_TIME_GAP_S = 1.0
MAXIMUM_INTERPOLATION_GAP_S = 0.2
NANOSECONDS_PER_SECOND = 1e9
STATE_INITIALIZING = "INITIALIZING"
STATE_INPUT_STALE = "INPUT_STALE"
STATE_GPS_ONLY = "GPS_ONLY"
STATE_TRACKING = "TRACKING"
SAVE_TIMING_PATTERN = re.compile(r"map save: (\w+) map written ([0-9.]+) s after the start of the save")
FIRST_SAVE_STAGE = "preliminary"


# ---------------------------------------------------------------- geometry

def wrap_angle(angle_rad):
    """Angle(s) wrapped to [-pi, pi)."""
    return (np.asarray(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def geodetic_to_plane(latitudes_deg, longitudes_deg, reference_latitude_deg, reference_longitude_deg):
    """(N, 2) array of east, north in metres around the reference point (rule 1)."""
    latitude_difference_rad = np.radians(np.asarray(latitudes_deg, dtype=np.float64) - reference_latitude_deg)
    longitude_difference_rad = np.radians(np.asarray(longitudes_deg, dtype=np.float64) - reference_longitude_deg)
    east = longitude_difference_rad * math.cos(math.radians(reference_latitude_deg)) * EARTH_RADIUS_M
    north = latitude_difference_rad * EARTH_RADIUS_M
    return np.column_stack([east, north])


def nearest_time_pairs(source_times, target_times, maximum_gap_s):
    """Pairs (source index, target index): the target nearest in time to each source, dropped when farther than the gap."""
    source_times = np.asarray(source_times, dtype=np.float64)
    target_times = np.asarray(target_times, dtype=np.float64)
    if source_times.size == 0 or target_times.size == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    order = np.argsort(target_times)
    sorted_targets = target_times[order]
    upper = np.clip(np.searchsorted(sorted_targets, source_times), 0, sorted_targets.size - 1)
    lower = np.clip(upper - 1, 0, sorted_targets.size - 1)
    use_lower = np.abs(source_times - sorted_targets[lower]) <= np.abs(sorted_targets[upper] - source_times)
    nearest = np.where(use_lower, lower, upper)
    is_close_enough = np.abs(sorted_targets[nearest] - source_times) <= maximum_gap_s
    return np.flatnonzero(is_close_enough), order[nearest[is_close_enough]]


def fit_rigid_transform_2d(moving_points, fixed_points):
    """Rotation matrix (2x2) and translation (2,) minimizing |R m + t - f|^2 (no scaling, no reflection)."""
    moving_centroid = moving_points.mean(axis=0)
    fixed_centroid = fixed_points.mean(axis=0)
    covariance = (moving_points - moving_centroid).T @ (fixed_points - fixed_centroid)
    left, _, right_transposed = np.linalg.svd(covariance)
    reflection_guard = np.diag([1.0, np.sign(np.linalg.det(right_transposed.T @ left.T)) or 1.0])
    rotation = right_transposed.T @ reflection_guard @ left.T
    translation = fixed_centroid - rotation @ moving_centroid
    return rotation, translation


def distance_statistics(differences):
    """RMS and maximum of the lengths of (N, k) difference vectors; None when there are none."""
    if len(differences) == 0:
        return {"count": 0, "rms_m": None, "max_m": None}
    distances = np.linalg.norm(differences, axis=1)
    return {"count": int(distances.size), "rms_m": float(math.sqrt(np.mean(distances ** 2))), "max_m": float(distances.max())}


def aligned_statistics(moving_points, fixed_points):
    """Rule 4: rigid fit of the moving points on the fixed points, then the statistics of what remains."""
    if len(moving_points) < 2:
        return distance_statistics([])
    rotation, translation = fit_rigid_transform_2d(moving_points, fixed_points)
    return distance_statistics(moving_points @ rotation.T + translation - fixed_points)


def interpolate_poses(sample_times, sample_poses, query_times, maximum_gap_s=MAXIMUM_INTERPOLATION_GAP_S):
    """Linear interpolation of (N, 4) poses (x, y, z, yaw; yaw the short way round) at the query times (rule 5).

    Returns (poses at the queries, boolean mask of the queries that have a sample within the gap before AND after)."""
    sample_times = np.asarray(sample_times, dtype=np.float64)
    query_times = np.asarray(query_times, dtype=np.float64)
    sample_count = sample_times.size
    if sample_count == 0:
        return np.zeros((query_times.size, 4)), np.zeros(query_times.size, dtype=bool)
    after_index = np.searchsorted(sample_times, query_times, side="left")
    before_index = np.searchsorted(sample_times, query_times, side="right") - 1
    has_neighbours = (before_index >= 0) & (after_index < sample_count)
    before_index = np.clip(before_index, 0, sample_count - 1)
    after_index = np.clip(after_index, 0, sample_count - 1)
    before_gap = query_times - sample_times[before_index]
    after_gap = sample_times[after_index] - query_times
    is_valid = has_neighbours & (before_gap <= maximum_gap_s) & (after_gap <= maximum_gap_s)
    interval = sample_times[after_index] - sample_times[before_index]
    weight = np.divide(before_gap, interval, out=np.zeros_like(before_gap), where=interval > 0.0)
    before_poses = sample_poses[before_index]
    after_poses = sample_poses[after_index]
    poses = before_poses + weight[:, None] * (after_poses - before_poses)
    poses[:, 3] = before_poses[:, 3] + weight * wrap_angle(after_poses[:, 3] - before_poses[:, 3])
    return poses, is_valid


# ---------------------------------------------------------------- files

def read_columns(csv_path, column_names):
    """Float columns of a CSV by header name -> dict of 1D arrays (a missing column is an error)."""
    with Path(csv_path).open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [name for name in column_names if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{csv_path}: missing columns {missing}")
        rows = [[float(row[name]) for name in column_names] for row in reader]
    values = np.array(rows, dtype=np.float64).reshape(-1, len(column_names))
    return {name: values[:, index] for index, name in enumerate(column_names)}


def read_gnss(csv_path):
    columns = read_columns(csv_path, ["bag_time_ns", "latitude", "longitude"])
    return {"times_s": columns["bag_time_ns"] / NANOSECONDS_PER_SECOND,
            "latitudes_deg": columns["latitude"], "longitudes_deg": columns["longitude"]}


def read_pose_csv(csv_path):
    """trajectory.csv and online_path.csv: stamp_sec, x, y, z, yaw (other columns are ignored)."""
    columns = read_columns(csv_path, ["stamp_sec", "x", "y", "z", "yaw"])
    return {"times_s": columns["stamp_sec"], "poses": np.column_stack([columns["x"], columns["y"], columns["z"], columns["yaw"]])}


def read_state_names(status_csv_path):
    with Path(status_csv_path).open(newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    return [float(row["stamp_sec"]) for row in rows], [row["state"] for row in rows]


def read_map_gps_origin(map_yaml_path):
    """(latitude, longitude) in degrees of the GNSS origin of a map.yaml, None when the map has none."""
    values = {}
    for line in Path(map_yaml_path).read_text().splitlines():
        key, separator, value = line.partition(":")
        if separator:
            values[key.strip()] = value.strip()
    if values.get("has_gps_origin", "false").lower() != "true":
        return None
    return float(values["origin_lat"]), float(values["origin_lon"])


def parse_save_timings(node_log_text):
    """Rule 7: one dict per save (a save starts at its 'preliminary' line): stage -> seconds after the start of the save."""
    saves = []
    for stage, seconds in SAVE_TIMING_PATTERN.findall(node_log_text):
        if stage == FIRST_SAVE_STAGE or not saves:
            saves.append({})
        saves[-1][stage] = float(seconds)
    return saves


# ---------------------------------------------------------------- evaluation

def safe_fraction(numerator, denominator):
    return None if denominator <= 0 else numerator / denominator


def state_fractions(state_names):
    """Rule 6 (counted in samples). A fraction with an empty denominator is None."""
    counts = {}
    for name in state_names:
        counts[name] = counts.get(name, 0) + 1
    total = len(state_names)
    initializing = counts.get(STATE_INITIALIZING, 0)
    stale = counts.get(STATE_INPUT_STALE, 0)
    fractions = {
        "input_stale_fraction": safe_fraction(stale, total - initializing),
        "gps_only_fraction": safe_fraction(counts.get(STATE_GPS_ONLY, 0), total - initializing - stale),
        "tracking_fraction": None,
    }
    if STATE_TRACKING in state_names:
        span = state_names[state_names.index(STATE_TRACKING):]
        span_stale = span.count(STATE_INPUT_STALE)
        fractions["tracking_fraction"] = safe_fraction(span.count(STATE_TRACKING), len(span) - span_stale)
    return {"state_counts": counts, **fractions}


def first_tracking_time(status_times, state_names):
    for stamp, name in zip(status_times, state_names):
        if name == STATE_TRACKING:
            return stamp
    return None


def yaw_difference_abs(first_yaw_rad, second_yaw_rad):
    return float(abs(wrap_angle(second_yaw_rad - first_yaw_rad)))


def evaluate_mapping(trajectory, gnss):
    """Rules 2 to 4 for the saved trajectory of a mapping run. `gnss` may be None (no gnss.csv)."""
    poses = trajectory["poses"]
    result = {"keyframe_rows": int(len(poses))}
    if len(poses) == 0:
        return result
    result["start_end_distance_m"] = float(np.linalg.norm(poses[-1, :2] - poses[0, :2]))
    result["start_end_yaw_difference_rad"] = yaw_difference_abs(poses[0, 3], poses[-1, 3])
    result["height_range_m"] = float(poses[:, 2].max() - poses[:, 2].min())
    if gnss is not None and len(gnss["times_s"]) > 0:
        plane = geodetic_to_plane(gnss["latitudes_deg"], gnss["longitudes_deg"], gnss["latitudes_deg"][0], gnss["longitudes_deg"][0])
        trajectory_indices, gnss_indices = nearest_time_pairs(trajectory["times_s"], gnss["times_s"], MAXIMUM_GNSS_TIME_GAP_S)
        result["gnss_aligned"] = aligned_statistics(poses[trajectory_indices, :2], plane[gnss_indices])
    return result


def evaluate_localization(online_path, map_trajectory, gnss, map_gps_origin, first_tracking_stamp):
    """Rule 5. Nothing is compared before the first TRACKING (None: the run never tracked)."""
    result = {"first_tracking_time_s": first_tracking_stamp}
    if first_tracking_stamp is None:
        return result
    row_times = map_trajectory["times_s"]
    is_row_in_scope = (row_times >= first_tracking_stamp) & (row_times >= row_times.min()) & (row_times <= row_times.max())
    scoped_times = row_times[is_row_in_scope]
    scoped_poses = map_trajectory["poses"][is_row_in_scope]
    online_poses, is_interpolated = interpolate_poses(online_path["times_s"], online_path["poses"], scoped_times)
    differences = online_poses[is_interpolated] - scoped_poses[is_interpolated]
    against_map = distance_statistics(differences[:, :2])
    against_map["height_difference_max_m"] = float(np.abs(differences[:, 2]).max()) if len(differences) else None
    result["against_map_trajectory"] = against_map
    if gnss is not None and len(gnss["times_s"]) > 0:
        result.update(evaluate_against_gnss(online_path, gnss, map_gps_origin, first_tracking_stamp))
    return result


def evaluate_against_gnss(online_path, gnss, map_gps_origin, first_tracking_stamp):
    is_after_tracking = gnss["times_s"] >= first_tracking_stamp
    gnss_times = gnss["times_s"][is_after_tracking]
    online_poses, is_interpolated = interpolate_poses(online_path["times_s"], online_path["poses"], gnss_times)
    online_xy = online_poses[is_interpolated, :2]
    latitudes = gnss["latitudes_deg"][is_after_tracking][is_interpolated]
    longitudes = gnss["longitudes_deg"][is_after_tracking][is_interpolated]
    first_latitude, first_longitude = gnss["latitudes_deg"][0], gnss["longitudes_deg"][0]
    result = {"against_gnss_aligned": aligned_statistics(online_xy, geodetic_to_plane(latitudes, longitudes, first_latitude, first_longitude))}
    if map_gps_origin is not None:
        origin_plane = geodetic_to_plane(latitudes, longitudes, *map_gps_origin)
        result["against_gnss_not_aligned"] = distance_statistics(online_xy - origin_plane)
    return result


def evaluate_directory(replay_directory, mode, map_directory):
    replay_directory = Path(replay_directory)
    gnss_path = replay_directory / "gnss.csv"
    gnss = read_gnss(gnss_path) if gnss_path.is_file() else None
    status_times, state_names = read_state_names(replay_directory / "status.csv")
    result = {"mode": mode, **state_fractions(state_names)}
    node_log_path = replay_directory / "node.log"
    if node_log_path.is_file():
        result["save_timings_s"] = parse_save_timings(node_log_path.read_text(errors="replace"))
    if mode == "mapping":
        trajectory_path = replay_directory / "map" / "trajectory.csv"
        if trajectory_path.is_file():
            result.update(evaluate_mapping(read_pose_csv(trajectory_path), gnss))
        return result
    map_directory = Path(map_directory)
    result.update(evaluate_localization(
        read_pose_csv(replay_directory / "online_path.csv"), read_pose_csv(map_directory / "trajectory.csv"), gnss,
        read_map_gps_origin(map_directory / "map.yaml"), first_tracking_time(status_times, state_names)))
    return result


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("replay_directory", type=Path, help="--out directory of replay_session.py")
    parser.add_argument("--mode", choices=("mapping", "localization"), help="default: the mode in summary.json")
    parser.add_argument("--map-directory", type=Path, help="localization: the directory of the map that was read (map.yaml, trajectory.csv)")
    parser.add_argument("--json-output", type=Path, help="default: <replay directory>/evaluation.json")
    return parser.parse_args(argv)


def main(argv=None):
    arguments = parse_arguments(argv)
    mode = arguments.mode or json.loads((arguments.replay_directory / "summary.json").read_text())["mode"]
    if mode == "localization" and arguments.map_directory is None:
        sys.exit("--map-directory is required for a localization run")
    result = evaluate_directory(arguments.replay_directory, mode, arguments.map_directory)
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    (arguments.json_output or arguments.replay_directory / "evaluation.json").write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
