#!/usr/bin/env python3
"""Replay a rosbag through localization_node (mapping or localization mode) inside the `replay` image.

Runs, in this order: the static TFs of the bag (re-published, because --start-offset skips them), localization_node
with use_sim_time (configure + activate), optionally foxglove_bridge, `ros2 bag play --clock --rate R --start-offset S`,
then SIGINT to the node so it saves map.pcd / map.yaml / trajectory.csv. Writes into --out:

    map/{map.pcd,map.yaml,trajectory.csv}   saved by the node (mapping mode)
    status.csv, online_path.csv             LocalizationStatus samples and the online map->base_footprint poses
    summary.json, node.log, play.log        numbers of the run (also printed at the end)
    map_topdown.png                         top-down render (render_map.py), mapping mode only

Normally started through run_replay.sh; see README.md.
"""

import argparse
import csv
import json
import math
import os
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lightnav_msgs.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_map  # noqa: E402

FORBIDDEN_DOMAIN_IDS = ("42", "43", "44")  # robot, GUI, tests
PLAYED_TOPICS = ("/velodyne_points", "/wheel/odom", "/tf", "/tf_static")
NODE_NAME = "localization_node"
POLL_PERIOD_S = 0.5
SERVICE_TIMEOUT_S = 30.0
FOXGLOVE_START_WAIT_S = 3.0
IDLE_SCAN_WAIT_S = 5.0  # the node counts as drained when no new aligned scan arrived for this long
DRAIN_TIMEOUT_S = 120.0
PLAY_EXIT_GRACE_S = 15.0
STATUS_STATE_NAMES = {0: "INITIALIZING", 1: "TRACKING", 2: "GPS_ONLY", 3: "ODOM_ONLY", 4: "LOST", 5: "INPUT_STALE"}


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
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
    parser.add_argument("--foxglove", action="store_true", help="start foxglove_bridge (no authentication: use on a trusted network)")
    parser.add_argument("--foxglove-port", type=int, default=8765)
    parser.add_argument("--hold-seconds", type=float, default=0.0,
                        help="keep the node and foxglove_bridge running this long after the replay, before the save")
    parser.add_argument("--save-timeout", type=float, default=600.0, help="seconds to wait for the map save after SIGINT")
    return parser.parse_args(argv)


def check_domain():
    domain_id = os.environ.get("ROS_DOMAIN_ID", "")
    if domain_id in FORBIDDEN_DOMAIN_IDS or not domain_id:
        sys.exit(f"ROS_DOMAIN_ID={domain_id!r} is not allowed for a replay (42 robot, 43 GUI, 44 tests); the replay image uses 61")


def find_node_executable():
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        candidate = Path(prefix) / "lib" / "lightnav_localization" / "localization_node"
        if candidate.is_file():
            return str(candidate)
    sys.exit("localization_node executable not found under AMENT_PREFIX_PATH")


def read_static_transforms(bag_directory):
    """First /tf_static message of the bag (it is recorded once at the start), or None."""
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static"]))
    if not reader.has_next():
        return None
    _, serialized, _ = reader.read_next()
    return deserialize_message(serialized, TFMessage)


def percentile(sorted_values, fraction):
    if not sorted_values:
        return float("nan")
    return sorted_values[min(len(sorted_values) - 1, int(math.ceil(fraction * len(sorted_values))) - 1)]


class Recorder(Node):
    """Collects what the node publishes during the replay. All callbacks run on the executor thread."""

    def __init__(self, republished_static_transforms):
        super().__init__("bag_replay_session")
        self.status_rows = []
        self.online_path_rows = []
        self.delivered_scan_stamps = []
        self.aligned_scan_count = 0
        self.last_aligned_scan_wall_s = time.monotonic()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_publisher = self.create_publisher(TFMessage, "/tf_static", latched)
        if republished_static_transforms is not None:
            self.static_publisher.publish(republished_static_transforms)
        self.create_subscription(
            LocalizationStatus, "/lightnav/localization/status",
            self._on_status, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, "/lightnav/localization/odom", self._on_odom, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        # raw=True: only the serialized bytes, so counting 1.5 MB scans does not cost a deserialization each.
        self.create_subscription(PointCloud2, "/velodyne_points", self._on_delivered_scan,
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE), raw=True)
        self.create_subscription(PointCloud2, "/lightnav/localization/aligned_scan", self._on_aligned_scan,
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)
        self.change_state_client = self.create_client(ChangeState, f"/{NODE_NAME}/change_state")

    def _on_status(self, message):
        stamp = message.header.stamp
        self.status_rows.append({
            "stamp_sec": stamp.sec + stamp.nanosec * 1e-9,
            "state": STATUS_STATE_NAMES.get(message.state, str(message.state)),
            "scan_ms": float(message.last_scan_time_ms),
            "match_ms": float(message.last_match_time_ms),
            "lidar_odom_inlier_ratio": float(message.last_lidar_odom_inlier_ratio),
            "odom_only_distance_m": float(message.odom_only_distance_m),
            "last_error": message.last_error,
        })

    def _on_odom(self, message):
        stamp = message.header.stamp
        pose = message.pose.pose
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                         1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
        self.online_path_rows.append((stamp.sec + stamp.nanosec * 1e-9, pose.position.x, pose.position.y, yaw))

    def _on_delivered_scan(self, serialized):
        # CDR: 4 byte encapsulation header, then header.stamp (int32 sec, uint32 nanosec).
        seconds, nanoseconds = struct.unpack_from("<iI", serialized, 4)
        self.delivered_scan_stamps.append(seconds + nanoseconds * 1e-9)

    def _on_aligned_scan(self, _serialized):
        self.aligned_scan_count += 1
        self.last_aligned_scan_wall_s = time.monotonic()


def change_state(recorder, transition_id):
    if not recorder.change_state_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
        raise RuntimeError(f"/{NODE_NAME}/change_state is not available")
    request = ChangeState.Request()
    request.transition.id = transition_id
    done = threading.Event()
    future = recorder.change_state_client.call_async(request)
    future.add_done_callback(lambda _: done.set())
    if not done.wait(SERVICE_TIMEOUT_S) or not future.result().success:
        raise RuntimeError(f"lifecycle transition {transition_id} failed (see node.log)")


def start_node(arguments, out_directory, log_file):
    command = [
        find_node_executable(), "--ros-args",
        "--params-file", str(arguments.params_file),
        "-p", "use_sim_time:=true",
        "-p", f"mode:={arguments.mode}",
        "-p", f"map_output_dir:={out_directory / 'map'}",
        "-p", f"status_rate_hz:={arguments.status_rate_hz}",
    ]
    if arguments.map_file:
        command += ["-p", f"map_file:={arguments.map_file}"]
    if arguments.map_yaml:
        command += ["-p", f"map_yaml:={arguments.map_yaml}"]
    for extra in arguments.node_param:
        command += ["-p", extra]
    return subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT)


def start_foxglove(arguments, log_file):
    command = ["ros2", "run", "foxglove_bridge", "foxglove_bridge", "--ros-args",
               "-p", f"port:={arguments.foxglove_port}", "-p", "address:=0.0.0.0", "-p", "use_sim_time:=true"]
    process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT)
    time.sleep(FOXGLOVE_START_WAIT_S)
    if process.poll() is not None:
        raise RuntimeError("foxglove_bridge exited at start (see foxglove.log)")
    return process


def start_player(arguments, log_file):
    command = ["ros2", "bag", "play", str(arguments.bag), "--clock", "100", "--rate", str(arguments.rate),
               "--start-offset", str(arguments.start_offset), "--read-ahead-queue-size", "2000",
               "--disable-keyboard-controls", "--topics", *PLAYED_TOPICS]
    if arguments.duration > 0.0:
        command += ["--playback-duration", str(arguments.duration)]
    return subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT)


def wait_until_node_is_drained(recorder, node_process):
    """The player only publishes; the node may still hold queued scans. Wait until aligned scans stop arriving."""
    deadline = time.monotonic() + DRAIN_TIMEOUT_S
    while time.monotonic() < deadline and node_process.poll() is None:
        if time.monotonic() - recorder.last_aligned_scan_wall_s > IDLE_SCAN_WAIT_S:
            return
        time.sleep(POLL_PERIOD_S)


def stop_process(process, timeout_s):
    if process.poll() is not None:
        return process.returncode
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait()


def write_csv(path, header, rows):
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)
        writer.writerows(rows)


def summarize(arguments, recorder, wall_seconds, node_exit_code, out_directory):
    scan_samples = sorted(row["scan_ms"] for row in recorder.status_rows if row["state"] != "INITIALIZING" and row["scan_ms"] > 0.0)
    states = {}
    for row in recorder.status_rows:
        states[row["state"]] = states.get(row["state"], 0) + 1
    errors = sorted({row["last_error"] for row in recorder.status_rows if row["last_error"]})
    delivered = len(recorder.delivered_scan_stamps)
    summary = {
        "mode": arguments.mode, "rate": arguments.rate, "start_offset_s": arguments.start_offset,
        "duration_s": arguments.duration, "wall_seconds": round(wall_seconds, 1), "node_exit_code": node_exit_code,
        "scans_delivered_to_session": delivered,
        "aligned_scans_published": recorder.aligned_scan_count,
        # The node starts the localizer after the first scans; the difference also holds those start-up scans and
        # best-effort losses of the aligned_scan topic itself, so read it as an upper bound of dropped scans.
        "scans_without_aligned_scan_upper_bound": delivered - recorder.aligned_scan_count,
        "scan_ms_p50": percentile(scan_samples, 0.5), "scan_ms_p95": percentile(scan_samples, 0.95),
        "scan_ms_max": scan_samples[-1] if scan_samples else float("nan"),
        "status_states": states, "last_errors": errors[-5:],
    }
    map_directory = out_directory / "map"
    if arguments.mode == "mapping" and (map_directory / "map.pcd").is_file() and (map_directory / "trajectory.csv").is_file():
        points = render_map.read_pcd_points(map_directory / "map.pcd")
        trajectory_xy = render_map.read_trajectory(map_directory / "trajectory.csv")
        summary.update(render_map.compute_statistics(points, trajectory_xy))
        png_path = out_directory / "map_topdown.png"
        render_map.render_png(points, trajectory_xy, png_path, 0.05, title=f"rate {arguments.rate:g}x, offset {arguments.start_offset:g} s")
        summary["png"] = str(png_path)
    elif arguments.mode == "mapping":
        summary["map_saved"] = False
    return summary


def main():
    arguments = parse_arguments()
    check_domain()
    out_directory = arguments.out
    (out_directory / "map").mkdir(parents=True, exist_ok=True)
    static_transforms = read_static_transforms(arguments.bag) if arguments.start_offset > 0.0 else None
    if arguments.start_offset > 0.0 and static_transforms is None:
        print("warning: the bag has no /tf_static message; the lidar extrinsic will not be found", file=sys.stderr)

    rclpy.init()
    recorder = Recorder(static_transforms)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(recorder)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    processes = []
    log_files = []
    node_exit_code = None
    started_wall_s = time.monotonic()
    try:
        node_log = (out_directory / "node.log").open("w")
        log_files.append(node_log)
        node_process = start_node(arguments, out_directory, node_log)
        processes.append(node_process)
        change_state(recorder, Transition.TRANSITION_CONFIGURE)
        change_state(recorder, Transition.TRANSITION_ACTIVATE)
        if arguments.foxglove:
            foxglove_log = (out_directory / "foxglove.log").open("w")
            log_files.append(foxglove_log)
            processes.append(start_foxglove(arguments, foxglove_log))
            print(f"foxglove_bridge: ws://<this machine>:{arguments.foxglove_port}", flush=True)

        play_log = (out_directory / "play.log").open("w")
        log_files.append(play_log)
        player = start_player(arguments, play_log)
        processes.append(player)
        play_status = player.wait()
        print(f"ros2 bag play finished with exit code {play_status}", flush=True)
        wait_until_node_is_drained(recorder, node_process)
        if arguments.hold_seconds > 0.0:
            print(f"holding {arguments.hold_seconds:g} s before the save (inspect in Foxglove)", flush=True)
            time.sleep(arguments.hold_seconds)
        node_exit_code = stop_process(node_process, arguments.save_timeout)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                stop_process(process, PLAY_EXIT_GRACE_S)
        executor.shutdown()
        recorder.destroy_node()
        rclpy.shutdown()
        for log_file in log_files:
            log_file.close()

    wall_seconds = time.monotonic() - started_wall_s
    write_csv(out_directory / "status.csv", ["stamp_sec", "state", "scan_ms", "match_ms", "lidar_odom_inlier_ratio", "odom_only_distance_m", "last_error"],
              [row.values() for row in recorder.status_rows])
    write_csv(out_directory / "online_path.csv", ["stamp_sec", "x", "y", "yaw"], recorder.online_path_rows)
    summary = summarize(arguments, recorder, wall_seconds, node_exit_code, out_directory)
    (out_directory / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if node_exit_code == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
