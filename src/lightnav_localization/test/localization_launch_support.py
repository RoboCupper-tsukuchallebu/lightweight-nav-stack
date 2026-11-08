"""Shared pieces of the launch tests of localization.launch.py (one test module per mode).

Each test module starts `ros2 launch lightnav_localization localization.launch.py mode:=<mode>` as a process, waits
until localization_node is ACTIVE and then sends SIGINT to that `ros2 launch` process only, like `docker compose stop`
does through tini (compose `stop_signal: SIGINT`). The node must then go through on_shutdown.
"""

import os
import signal
import struct
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch.actions
import launch_testing.actions
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

NODE_NAME = "localization_node"
ACTIVATION_TIMEOUT_SECONDS = 60.0
OUTPUT_TIMEOUT_SECONDS = 10
# launch_testing waits on stderr by default, but the output of the nested ros2 launch arrives on stdout.
OUTPUT_STREAM = "stdout"
SERVICE_TIMEOUT_SECONDS = 1.0
WHEEL_PERIOD_SECONDS = 0.02
SCAN_PERIOD_SECONDS = 0.2
ROOM_HALF_SIZE_M = 10.0
ROOM_GRID_STEP_M = 0.25
ROOM_HEIGHT_M = 2.0
FLOAT32_BYTES = 4
POINT_FIELD_NAMES = ("x", "y", "z", "time")
LAUNCH_SIGTERM_TIMEOUT_SECONDS = 15
LAUNCH_SIGKILL_TIMEOUT_SECONDS = 10


def read_all_output_text(proc_output):
    return "".join(output.text.decode(errors="replace") for output in proc_output)


def build_room_points():
    """Points on the floor and the four walls of a square room around the origin."""
    steps = int(2 * ROOM_HALF_SIZE_M / ROOM_GRID_STEP_M) + 1
    axis = [-ROOM_HALF_SIZE_M + index * ROOM_GRID_STEP_M for index in range(steps)]
    heights = [index * ROOM_GRID_STEP_M for index in range(int(ROOM_HEIGHT_M / ROOM_GRID_STEP_M) + 1)]
    points = [(x, y, -0.2) for x in axis for y in axis]
    for along in axis:
        for height in heights:
            points.extend(
                [
                    (along, -ROOM_HALF_SIZE_M, height),
                    (along, ROOM_HALF_SIZE_M, height),
                    (-ROOM_HALF_SIZE_M, along, height),
                    (ROOM_HALF_SIZE_M, along, height),
                ]
            )
    return points


def write_binary_pcd(file_path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n"
    )
    with open(file_path, "wb") as stream:
        stream.write(header.encode("ascii"))
        stream.writelines(struct.pack("<fff", *point) for point in points)


def build_launch_description(mode):
    """Returns (LaunchDescription, {"launch_process": ExecuteProcess}, work directory) for launch_testing."""
    work_directory = Path(tempfile.mkdtemp(prefix=f"lightnav_localization_launch_{mode}_"))
    output_directory = work_directory / "map_out"
    map_file = work_directory / "map.pcd"
    write_binary_pcd(map_file, build_room_points())
    params_file = work_directory / "params.yaml"
    # Short waits so the test is quick; everything else keeps the package defaults.
    params_file.write_text(
        f"{NODE_NAME}:\n"
        "  ros__parameters:\n"
        f"    map_output_dir: {output_directory}\n"
        "    allow_mapping_without_gps: true\n"
        "    gps_timeout_s: 1.0\n"
        "    keyframe_time_s: 0.5\n"
        "    status_rate_hz: 5.0\n"
    )
    command = [
        "ros2", "launch", "lightnav_localization", "localization.launch.py",
        f"mode:={mode}",
        f"params_file:={params_file}",
        f"sigterm_timeout:={LAUNCH_SIGTERM_TIMEOUT_SECONDS}",
        f"sigkill_timeout:={LAUNCH_SIGKILL_TIMEOUT_SECONDS}",
    ]
    if mode == "localization":
        command.append(f"map_file:={map_file}")
    launch_process = launch.actions.ExecuteProcess(cmd=command, output="screen")
    description = launch.LaunchDescription([launch_process, launch_testing.actions.ReadyToTest()])
    return description, {"launch_process": launch_process, "output_directory": output_directory}


class LocalizationLaunchCase(unittest.TestCase):
    """Base class: a rclpy helper node, the lifecycle wait and the synthetic inputs."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.helper_node = rclpy.create_node("test_localization_launch_helper")
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cls.static_tf_publisher = cls.helper_node.create_publisher(TFMessage, "/tf_static", latched)
        cls.wheel_publisher = cls.helper_node.create_publisher(Odometry, "/wheel/odom", reliable)
        cls.points_publisher = cls.helper_node.create_publisher(PointCloud2, "/velodyne_points", reliable)
        cls.room_cloud_data = b"".join(struct.pack("<ffff", x, y, z, 0.0) for x, y, z in build_room_points())
        cls.room_point_count = len(cls.room_cloud_data) // (FLOAT32_BYTES * len(POINT_FIELD_NAMES))

    @classmethod
    def tearDownClass(cls):
        cls.helper_node.destroy_node()
        rclpy.shutdown()

    def read_lifecycle_state_id(self):
        """The lifecycle state id of the node, or None while its get_state service does not answer."""
        client = self.helper_node.create_client(GetState, f"/{NODE_NAME}/get_state")
        try:
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
                return None
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.helper_node, future, timeout_sec=SERVICE_TIMEOUT_SECONDS)
            return future.result().current_state.id if future.done() else None
        finally:
            self.helper_node.destroy_client(client)

    def wait_until_active(self):
        deadline = time.monotonic() + ACTIVATION_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            if self.read_lifecycle_state_id() == State.PRIMARY_STATE_ACTIVE:
                return True
        return False

    def publish_lidar_extrinsic(self):
        transform = TransformStamped()
        transform.header.stamp = self.helper_node.get_clock().now().to_msg()
        transform.header.frame_id = "base_footprint"
        transform.child_frame_id = "velodyne"
        transform.transform.rotation.w = 1.0
        self.static_tf_publisher.publish(TFMessage(transforms=[transform]))

    def publish_wheel_odom(self):
        message = Odometry()
        message.header.stamp = self.helper_node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_footprint"
        message.pose.pose.orientation.w = 1.0
        self.wheel_publisher.publish(message)

    def publish_scan(self):
        cloud = PointCloud2()
        cloud.header.stamp = self.helper_node.get_clock().now().to_msg()
        cloud.header.frame_id = "velodyne"
        cloud.height = 1
        cloud.width = self.room_point_count
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.point_step = FLOAT32_BYTES * len(POINT_FIELD_NAMES)
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name=name, offset=index * FLOAT32_BYTES, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(POINT_FIELD_NAMES)
        ]
        cloud.data = self.room_cloud_data
        self.points_publisher.publish(cloud)

    def publish_inputs_until(self, is_done, timeout_seconds):
        """Publishes a still robot in a room (wheel 50 Hz, scan 5 Hz) until is_done() or the timeout."""
        deadline = time.monotonic() + timeout_seconds
        next_scan = time.monotonic()
        while time.monotonic() < deadline:
            self.publish_wheel_odom()
            if time.monotonic() >= next_scan:
                self.publish_scan()
                next_scan += SCAN_PERIOD_SECONDS
            rclpy.spin_once(self.helper_node, timeout_sec=WHEEL_PERIOD_SECONDS)
            if is_done():
                return True
        return is_done()

    def send_sigint_to(self, launch_process):
        os.kill(launch_process.process_details["pid"], signal.SIGINT)
