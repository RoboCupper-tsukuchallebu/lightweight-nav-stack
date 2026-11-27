"""Arguments of replay_session.py and the pure functions behind them (no ROS import: testable anywhere)."""

import argparse
import csv
import math
from pathlib import Path

MAXIMUM_INITIAL_POSE_GAP_S = 2.0  # rows of the map trajectory further apart than this are not interpolated
DEFAULT_SAVE_TIMEOUT_S = 600.0


class InitialPoseError(ValueError):
    """The initial pose cannot be taken from the given text or trajectory."""


def parse_initial_pose(text):
    """'X,Y,YAW' (m, m, rad) -> (x, y, yaw). Raises argparse.ArgumentTypeError (usable as an argparse type)."""
    parts = text.split(",")
    try:
        values = tuple(float(part) for part in parts)
    except ValueError:
        values = ()
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError(f"--initial-pose wants three finite numbers X,Y,YAW (m, m, rad), got '{text}'")
    return values


def wrap_angle(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def read_trajectory_rows(csv_path):
    """Rows (stamp_sec, x, y, yaw) of a saved trajectory.csv, in file order."""
    with Path(csv_path).open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [name for name in ("stamp_sec", "x", "y", "yaw") if name not in (reader.fieldnames or [])]
        if missing:
            raise InitialPoseError(f"{csv_path}: missing columns {missing}")
        return [(float(row["stamp_sec"]), float(row["x"]), float(row["y"]), float(row["yaw"])) for row in reader]


def interpolate_initial_pose(rows, query_time_s, maximum_gap_s=MAXIMUM_INITIAL_POSE_GAP_S):
    """Pose at `query_time_s` from the two rows around it: position linear, yaw the short way round.

    Raises InitialPoseError when the time is outside the rows or the two rows are more than the gap apart.
    Returns a dict: x, y, yaw, query_time_s, before_stamp_s, after_stamp_s."""
    if len(rows) < 2:
        raise InitialPoseError("the trajectory needs at least two rows")
    for before, after in zip(rows, rows[1:]):
        if before[0] <= query_time_s <= after[0]:
            break
    else:
        raise InitialPoseError(
            f"start time {query_time_s:.3f} s is outside the trajectory ({rows[0][0]:.3f} to {rows[-1][0]:.3f} s)")
    interval = after[0] - before[0]
    if interval > maximum_gap_s:
        raise InitialPoseError(
            f"the trajectory rows around the start time are {interval:.3f} s apart (more than {maximum_gap_s:g} s)")
    weight = 0.0 if interval <= 0.0 else (query_time_s - before[0]) / interval
    return {
        "x": before[1] + weight * (after[1] - before[1]),
        "y": before[2] + weight * (after[2] - before[2]),
        "yaw": wrap_angle(before[3] + weight * wrap_angle(after[3] - before[3])),
        "query_time_s": query_time_s, "before_stamp_s": before[0], "after_stamp_s": after[0],
    }


def played_topics(sensor_topics, extra_topics, record_gnss_topic):
    """Topics for `ros2 bag play --topics`: the sensor topics, the extra ones and the recorded GNSS topic, each once."""
    topics = list(sensor_topics)
    for topic in [*extra_topics, record_gnss_topic]:
        if topic and topic not in topics:
            topics.append(topic)
    return topics


def build_parser(description):
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", type=Path, default=Path("/bag"), help="rosbag2 directory (mcap)")
    parser.add_argument("--out", type=Path, default=Path("/out"), help="output directory")
    parser.add_argument("--mode", choices=("mapping", "localization"), default="mapping")
    parser.add_argument("--rate", type=float, default=1.0, help="playback rate for ros2 bag play -r")
    parser.add_argument("--start-offset", type=float, default=0.0, help="seconds to skip from the bag start")
    parser.add_argument("--duration", type=float, default=0.0, help="stop playback after this many bag seconds (0 = to the end)")
    parser.add_argument("--params-file", type=Path, default=Path("/ws/config/localization_indoor.yaml"))
    parser.add_argument("--map-file", help="prior map.pcd (localization mode)")
    parser.add_argument("--map-yaml", help="prior map.yaml (localization mode)")
    parser.add_argument("--status-rate-hz", type=float, default=10.0, help="LocalizationStatus rate (more samples for scan_ms)")
    parser.add_argument("--node-param", action="append", default=[], metavar="NAME:=VALUE",
                        help="extra localization_node parameter (repeatable)")
    parser.add_argument("--extra-topic", action="append", default=[], metavar="TOPIC",
                        help="bag topic to play besides the sensor topics, e.g. /fix_all for GNSS (repeatable)")
    parser.add_argument("--record-gnss-topic", metavar="TOPIC",
                        help="play this NavSatFix topic and write what is received to gnss.csv (the node uses it only when its gps_topic is the same)")
    parser.add_argument("--save-map-service", action="store_true",
                        help="mapping only: after the replay, before the stop, call ~/save_map and write its answer to summary.json")
    parser.add_argument("--initial-pose", type=parse_initial_pose, metavar="X,Y,YAW",
                        help="localization only: call ~/set_initial_pose (m, m, rad) once the node is active; exit non-zero without replaying when it is not accepted")
    parser.add_argument("--initial-pose-from-trajectory", type=Path, metavar="FILE",
                        help="localization only: initial pose interpolated from the trajectory.csv of the map at the start time "
                             "(bag start + --start-offset); an error outside the map's times or when the two rows are over 2 s apart")
    parser.add_argument("--foxglove", action="store_true", help="start foxglove_bridge (no authentication: use on a trusted network)")
    parser.add_argument("--foxglove-port", type=int, default=8765)
    parser.add_argument("--hold-seconds", type=float, default=0.0,
                        help="keep the node and foxglove_bridge running this long after the replay, before the save")
    parser.add_argument("--save-timeout", type=float, default=DEFAULT_SAVE_TIMEOUT_S,
                        help="seconds to wait for the map save (the save_map call, and the save after SIGINT)")
    return parser


def validate_arguments(parser, arguments):
    """Exits through parser.error() on a combination that cannot work."""
    if arguments.save_timeout <= 0.0:
        parser.error("--save-timeout must be positive")
    if arguments.save_map_service and arguments.mode != "mapping":
        parser.error("--save-map-service is only for --mode mapping")
    wants_initial_pose = arguments.initial_pose is not None or arguments.initial_pose_from_trajectory is not None
    if wants_initial_pose and arguments.mode != "localization":
        parser.error("--initial-pose and --initial-pose-from-trajectory are only for --mode localization")
    if arguments.initial_pose is not None and arguments.initial_pose_from_trajectory is not None:
        parser.error("--initial-pose and --initial-pose-from-trajectory cannot be used together")


def parse_arguments(description, argv=None):
    parser = build_parser(description)
    arguments = parser.parse_args(argv)
    validate_arguments(parser, arguments)
    return arguments
