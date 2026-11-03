"""CLI: waypoints YAML を検証して FollowWaypoints goal を送る。不正 YAML は非 0 終了。"""

import argparse
import signal
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from lightnav_mission.waypoint_files import WaypointFileError, load_waypoints_file
from lightnav_msgs.action import FollowWaypoints
from lightnav_msgs.msg import Waypoint

EXIT_OK = 0
EXIT_FAILED = 1
SERVER_WAIT_S = 5.0
CANCEL_WAIT_S = 2.0
ACCEPT_WAIT_S = 5.0
SPIN_SLICE_S = 0.1
UNCONFIRMED_CANCEL_MESSAGE = "Ctrl-C: cancel を確認できなかった。ロボットが走行中の可能性"


def _spin_until_done(node, executor, future, timeout_s=None, interrupted=None) -> None:
    """短い単位で spin する。SignalHandlerOptions.NO では待機中の SIGINT で wait が起きず、
    Python のシグナル処理が回らないため、定期的に Python に制御を戻す。"""
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    while not future.done():
        if interrupted is not None and interrupted.requested:
            return
        remaining_s = None if deadline is None else deadline - time.monotonic()
        if remaining_s is not None and remaining_s <= 0:
            return
        rclpy.spin_until_future_complete(
            node, future, executor=executor,
            timeout_sec=SPIN_SLICE_S if remaining_s is None else min(SPIN_SLICE_S, remaining_s))


def _cancel_goal_after_interrupt(node, executor, goal_handle) -> None:
    """Ctrl-C ではロボットを走らせたまま終わらず、goal を cancel する。"""
    try:
        cancel_future = goal_handle.cancel_goal_async()
        _spin_until_done(node, executor, cancel_future, CANCEL_WAIT_S)
        if cancel_future.done() and cancel_future.result().goals_canceling:
            print("Ctrl-C: goal を cancel した", file=sys.stderr)
            return
    except RuntimeError as error:
        print(f"Ctrl-C: cancel エラー: {error}", file=sys.stderr)
    print(UNCONFIRMED_CANCEL_MESSAGE, file=sys.stderr)


def _handle_interrupt_before_acceptance(node, executor, send_future) -> int:
    """受理応答を期限まで待ち、受理されていれば cancel を確認する。"""
    try:
        _spin_until_done(node, executor, send_future, ACCEPT_WAIT_S)
        if not send_future.done():
            print(UNCONFIRMED_CANCEL_MESSAGE, file=sys.stderr)
        elif send_future.result().accepted:
            _cancel_goal_after_interrupt(node, executor, send_future.result())
        else:
            print("Ctrl-C: goal は reject されていた", file=sys.stderr)
    except RuntimeError as error:
        print(f"{UNCONFIRMED_CANCEL_MESSAGE}: {error}", file=sys.stderr)
    return EXIT_FAILED


def main(argv=None) -> int:
    interrupted = SimpleNamespace(requested=False)

    def handle_interrupt(_signal_number, _frame):
        interrupted.requested = True

    previous_handler = signal.signal(signal.SIGINT, handle_interrupt)
    try:
        exit_code = _run(argv, interrupted)
        if interrupted.requested:
            print("Ctrl-C: 終了します", file=sys.stderr)
            return EXIT_FAILED
        return exit_code
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def _run(argv, interrupted) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("waypoints_yaml")
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--action-name", default="/lightnav/mission/follow_waypoints")
    arguments = parser.parse_args(argv)

    try:
        records = load_waypoints_file(arguments.waypoints_yaml)
    except WaypointFileError as error:
        print(f"YAML 不正: {error}", file=sys.stderr)
        return EXIT_FAILED
    if not 0 <= arguments.start_index < len(records):
        print("start-index が範囲外", file=sys.stderr)
        return EXIT_FAILED

    # 既定のシグナルハンドラは SIGINT で context を shutdown し、cancel が送れなくなる
    context = rclpy.Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("send_waypoints", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = ActionClient(node, FollowWaypoints, arguments.action_name)
    try:
        server_deadline = time.monotonic() + SERVER_WAIT_S
        while not client.server_is_ready():
            if interrupted.requested:
                return EXIT_FAILED
            remaining_s = server_deadline - time.monotonic()
            if remaining_s <= 0:
                print("action サーバが見つからない", file=sys.stderr)
                return EXIT_FAILED
            executor.spin_once(timeout_sec=min(SPIN_SLICE_S, remaining_s))
        goal = FollowWaypoints.Goal()
        goal.waypoints = [
            Waypoint(x=record.x, y=record.y, arrival_radius_m=record.arrival_radius_m)
            for record in records
        ]
        goal.start_index = arguments.start_index
        if interrupted.requested:
            return EXIT_FAILED
        send_future = client.send_goal_async(
            goal, feedback_callback=lambda message: node.get_logger().info(
                f"{message.feedback.state} wp={message.feedback.current_waypoint_index}"
                f" dist={message.feedback.distance_to_waypoint_m:.2f}"
            ),
        )
        _spin_until_done(node, executor, send_future, interrupted=interrupted)
        if interrupted.requested:
            return _handle_interrupt_before_acceptance(node, executor, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("goal が reject された", file=sys.stderr)
            return EXIT_FAILED
        result_future = goal_handle.get_result_async()
        _spin_until_done(node, executor, result_future, interrupted=interrupted)
        if interrupted.requested:
            _cancel_goal_after_interrupt(node, executor, goal_handle)
            return EXIT_FAILED
        result = result_future.result().result
        print(f"success={result.success} reached={result.reached_count} {result.message}")
        return EXIT_OK if result.success else EXIT_FAILED
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
