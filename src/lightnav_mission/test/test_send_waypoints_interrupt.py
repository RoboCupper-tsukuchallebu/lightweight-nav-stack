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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

import lightnav_msgs
from lightnav_msgs.action import FollowWaypoints

INSTALL_LIBRARY_DIRECTORY = pathlib.Path(lightnav_msgs.__file__).resolve().parents[3]
TEST_DOMAIN_ID = "48"
GOAL_START_TIMEOUT_S = 15.0
PROCESS_EXIT_TIMEOUT_S = 10.0
CANCEL_OBSERVE_TIMEOUT_S = 5.0
EXECUTE_DEADLINE_S = 20.0
SLOW_ACCEPT_DELAY_S = 1.0


class FakeFollowWaypointsServer:
    """goal 受理を accept_delay_s だけ遅らせ、cancel が届くまで走り続ける偽サーバ。"""

    def __init__(self, context, accept_delay_s):
        self.node = rclpy.create_node("fake_follow_waypoints_server", context=context)
        self.accept_delay_s = accept_delay_s
        self.goal_received = threading.Event()
        self.goal_running = threading.Event()
        self.cancel_observed = threading.Event()
        ActionServer(
            self.node, FollowWaypoints, "/lightnav/mission/follow_waypoints",
            goal_callback=self._on_goal, execute_callback=self._execute,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT)
        self.executor = MultiThreadedExecutor(num_threads=4, context=context)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def _on_goal(self, _goal_request):
        self.goal_received.set()
        time.sleep(self.accept_delay_s)
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self.goal_running.set()
        deadline = time.monotonic() + EXECUTE_DEADLINE_S
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.02)
        if not goal_handle.is_cancel_requested:
            goal_handle.abort()  # cancel が届かなかった場合も有限時間で終わる
            return FollowWaypoints.Result()
        self.cancel_observed.set()
        goal_handle.canceled()
        return FollowWaypoints.Result()

    def shutdown(self):
        self.executor.shutdown()
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()


def _start_cli(waypoints_file):
    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": TEST_DOMAIN_ID,
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        # macOS の SIP は子プロセスへ DYLD_LIBRARY_PATH を渡さないので、msg の dylib の場所を明示する
        "DYLD_LIBRARY_PATH": str(INSTALL_LIBRARY_DIRECTORY),
    }
    return subprocess.Popen(
        [sys.executable, "-m", "lightnav_mission.send_waypoints", str(waypoints_file)],
        env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)


def _interrupt_and_check(tmp_path, accept_delay_s, interrupt_after):
    """interrupt_after(server) を待ってから SIGINT を送り、cancel が届くことを確かめる。"""
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=int(TEST_DOMAIN_ID))
    server = FakeFollowWaypointsServer(context, accept_delay_s)
    waypoints_file = tmp_path / "waypoints.yaml"
    waypoints_file.write_text(yaml.safe_dump({"waypoints": [{"x": 1.0, "y": 0.0}]}))
    process = _start_cli(waypoints_file)
    try:
        assert interrupt_after(server).wait(GOAL_START_TIMEOUT_S), "goal が届かなかった"
        process.send_signal(signal.SIGINT)
        try:
            exit_code = process.wait(timeout=PROCESS_EXIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.kill()
            raise AssertionError("CLI が SIGINT 後に終了しなかった: " + process.communicate()[1])
        standard_error = process.stderr.read()
        assert exit_code == 1, standard_error
        assert server.cancel_observed.wait(CANCEL_OBSERVE_TIMEOUT_S), standard_error
        assert "goal を cancel した" in standard_error
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stderr.close()
        server.shutdown()
        context.try_shutdown()


def test_sigint_while_goal_is_running_cancels_goal(tmp_path):
    _interrupt_and_check(tmp_path, 0.0, lambda server: server.goal_running)


def test_sigint_while_goal_acceptance_is_pending_still_cancels_goal(tmp_path):
    _interrupt_and_check(tmp_path, SLOW_ACCEPT_DELAY_S, lambda server: server.goal_received)
