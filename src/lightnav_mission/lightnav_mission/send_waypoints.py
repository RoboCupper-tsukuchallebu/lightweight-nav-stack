"""CLI: waypoints YAML を検証して FollowWaypoints goal を送る。不正 YAML は非 0 終了。"""

import argparse
import sys

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


def main(argv=None) -> int:
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
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            print("action サーバが見つからない", file=sys.stderr)
            return EXIT_FAILED
        goal = FollowWaypoints.Goal()
        goal.waypoints = [
            Waypoint(x=record.x, y=record.y, arrival_radius_m=record.arrival_radius_m)
            for record in records
        ]
        goal.start_index = arguments.start_index
        send_future = client.send_goal_async(
            goal, feedback_callback=lambda message: node.get_logger().info(
                f"{message.feedback.state} wp={message.feedback.current_waypoint_index}"
                f" dist={message.feedback.distance_to_waypoint_m:.2f}"
            ),
        )
        rclpy.spin_until_future_complete(node, send_future, executor=executor)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("goal が reject された", file=sys.stderr)
            return EXIT_FAILED
        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result_future, executor=executor)
        except KeyboardInterrupt:
            # Ctrl-C ではロボットを走らせたまま終わらず、goal を cancel する
            # 中断された executor は内部状態が壊れ得るので、cancel 用に作り直して回す
            executor.shutdown()
            cancel_executor = SingleThreadedExecutor(context=context)
            cancel_executor.add_node(node)
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                node, cancel_future, executor=cancel_executor, timeout_sec=CANCEL_WAIT_S)
            cancel_executor.shutdown()
            if cancel_future.done():
                print("Ctrl-C: goal を cancel した", file=sys.stderr)
            else:
                print("Ctrl-C: cancel を確認できなかった。ロボットが走行中の可能性", file=sys.stderr)
            return EXIT_FAILED
        result = result_future.result().result
        print(f"success={result.success} reached={result.reached_count} {result.message}")
        return EXIT_OK if result.success else EXIT_FAILED
    except KeyboardInterrupt:
        return EXIT_FAILED
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
