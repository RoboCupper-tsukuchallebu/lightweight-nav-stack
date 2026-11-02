"""send_waypoints を別プロセスで起動し SIGINT を送ると、goal を cancel して終了コード 1 になる。"""

import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor

import lightnav_msgs
from lightnav_msgs.action import FollowWaypoints

INSTALL_LIBRARY_DIRECTORY = pathlib.Path(lightnav_msgs.__file__).resolve().parents[3]
TEST_DOMAIN_ID = "48"
GOAL_START_TIMEOUT_S = 15.0
PROCESS_EXIT_TIMEOUT_S = 10.0
CANCEL_OBSERVE_TIMEOUT_S = 5.0
EXECUTE_DEADLINE_S = 20.0


def test_sigint_cancels_goal_and_cli_exits_in_bounded_time(tmp_path):
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=int(TEST_DOMAIN_ID))
    server_node = rclpy.create_node("fake_follow_waypoints_server", context=context)
    cancel_observed = threading.Event()
    goal_running = threading.Event()

    def execute(goal_handle):
        goal_running.set()
        deadline = time.monotonic() + EXECUTE_DEADLINE_S
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            threading.Event().wait(0.02)
        if not goal_handle.is_cancel_requested:
            goal_handle.abort()  # cancel が届かなかった場合も有限時間で終わる
            return FollowWaypoints.Result()
        cancel_observed.set()
        goal_handle.canceled()
        return FollowWaypoints.Result()

    ActionServer(
        server_node, FollowWaypoints, "/lightnav/mission/follow_waypoints",
        execute_callback=execute, cancel_callback=lambda _handle: CancelResponse.ACCEPT)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    waypoints_file = tmp_path / "waypoints.yaml"
    waypoints_file.write_text(yaml.safe_dump({"waypoints": [{"x": 1.0, "y": 0.0}]}))
    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": TEST_DOMAIN_ID,
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        # macOS の SIP は子プロセスへ DYLD_LIBRARY_PATH を渡さないので、msg の dylib の場所を明示する
        "DYLD_LIBRARY_PATH": str(INSTALL_LIBRARY_DIRECTORY),
    }
    process = subprocess.Popen(
        [sys.executable, "-m", "lightnav_mission.send_waypoints", str(waypoints_file)],
        env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        assert goal_running.wait(GOAL_START_TIMEOUT_S), "goal が実行開始しなかった"
        process.send_signal(signal.SIGINT)
        try:
            exit_code = process.wait(timeout=PROCESS_EXIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.kill()
            raise AssertionError("CLI が SIGINT 後に終了しなかった: " + process.communicate()[1])
        standard_error = process.stderr.read()
        assert exit_code == 1, standard_error
        assert cancel_observed.wait(CANCEL_OBSERVE_TIMEOUT_S)
        assert "goal を cancel した" in standard_error
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stderr.close()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        server_node.destroy_node()
        context.try_shutdown()
