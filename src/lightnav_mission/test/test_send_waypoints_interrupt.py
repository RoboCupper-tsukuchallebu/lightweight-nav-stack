"""send_waypoints を別プロセスで起動し SIGINT を送ると、goal を cancel して終了コード 1 になる。"""

import os
import pathlib
import signal
import subprocess
import sys
import threading
import time
import uuid

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

    def __init__(
            self, context, action_name, accept_delay_s, cancel_response=CancelResponse.ACCEPT):
        self.node = rclpy.create_node("fake_follow_waypoints_server", context=context)
        self.accept_delay_s = accept_delay_s
        self.goal_received = threading.Event()
        self.goal_running = threading.Event()
        self.cancel_observed = threading.Event()
        self.cancel_received = threading.Event()
        self.cancel_response = cancel_response
        self.stop_execution = threading.Event()
        self.execution_finished = threading.Event()
        ActionServer(
            self.node, FollowWaypoints, action_name,
            goal_callback=self._on_goal, execute_callback=self._execute,
            cancel_callback=self._on_cancel)
        self.executor = MultiThreadedExecutor(num_threads=4, context=context)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def _on_goal(self, _goal_request):
        self.goal_received.set()
        time.sleep(self.accept_delay_s)
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self.cancel_received.set()
        return self.cancel_response

    def _execute(self, goal_handle):
        self.goal_running.set()
        deadline = time.monotonic() + EXECUTE_DEADLINE_S
        try:
            while (not goal_handle.is_cancel_requested
                   and not self.stop_execution.is_set()
                   and time.monotonic() < deadline):
                time.sleep(0.02)
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
                return FollowWaypoints.Result()
            self.cancel_observed.set()
            goal_handle.canceled()
            return FollowWaypoints.Result()
        finally:
            self.execution_finished.set()

    def shutdown(self):
        self.stop_execution.set()
        if self.goal_received.is_set():
            assert self.execution_finished.wait(CANCEL_OBSERVE_TIMEOUT_S)
        self.executor.shutdown()
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()


def _start_cli(waypoints_file, action_name):
    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": TEST_DOMAIN_ID,
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        # macOS の SIP は子プロセスへ DYLD_LIBRARY_PATH を渡さないので、msg の dylib の場所を明示する
        "DYLD_LIBRARY_PATH": str(INSTALL_LIBRARY_DIRECTORY),
    }
    return subprocess.Popen(
        [sys.executable, "-m", "lightnav_mission.send_waypoints", str(waypoints_file),
         "--action-name", action_name],
        env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)


def _interrupt_and_check(
        tmp_path, accept_delay_s, interrupt_after, cancel_response=CancelResponse.ACCEPT):
    """interrupt_after(server) を待ってから SIGINT を送り、cancel が届くことを確かめる。"""
    # 試験ごとに action 名を変える。同じ名前だと、同じプロセスで 2 つ目以降の試験の CLI が送った goal が
    # 15 秒待っても偽サーバに届かないことがある（負荷が高いとき。#32）。1 つ目の試験は落ちず、名前を
    # 変えると落ちなくなるので、直前の試験の endpoint が discovery に残って混ざるとみている。
    action_name = f"/lightnav/mission/follow_waypoints_test_{uuid.uuid4().hex}"
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=int(TEST_DOMAIN_ID))
    server = FakeFollowWaypointsServer(context, action_name, accept_delay_s, cancel_response)
    waypoints_file = tmp_path / "waypoints.yaml"
    waypoints_file.write_text(yaml.safe_dump({"waypoints": [{"x": 1.0, "y": 0.0}]}))
    process = _start_cli(waypoints_file, action_name)
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
        if cancel_response == CancelResponse.REJECT:
            assert server.cancel_received.wait(CANCEL_OBSERVE_TIMEOUT_S), standard_error
            assert not server.cancel_observed.is_set(), standard_error
            assert "走行中の可能性" in standard_error
            assert "goal を cancel した" not in standard_error
        else:
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


def test_sigint_when_cancel_is_rejected_warns_goal_may_still_be_running(tmp_path):
    _interrupt_and_check(
        tmp_path, 0.0, lambda server: server.goal_running, CancelResponse.REJECT)
