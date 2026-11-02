"""Child process for the real-service tests: runs the lifecycle against a real PlanPath
service and prints one JSON object. Not collected by pytest (name has no test_ prefix)."""

import json
import sys
import time

import rclpy
from lightnav_planner.planner_node import SERVICE_NAME, PlannerNode
from rclpy.executors import SingleThreadedExecutor

from lightnav_msgs.msg import Waypoint
from lightnav_msgs.srv import PlanPath

CALL_TIMEOUT_S = 5.0
GRAPH_SETTLE_TIMEOUT_S = 5.0


def call_plan_path(planner_node, client_node, client):
    request = PlanPath.Request()
    request.frame_id = "map"
    request.start.header.frame_id = "map"
    request.waypoints = [Waypoint(x=0.0, y=1.0)]
    executor = SingleThreadedExecutor()
    executor.add_node(planner_node)
    executor.add_node(client_node)
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=CALL_TIMEOUT_S)
    executor.remove_node(client_node)
    executor.remove_node(planner_node)
    response = future.result() if future.done() else None
    return {
        "answered": response is not None,
        "success": bool(response and response.success),
        "waypoint_indices": list(response.waypoint_indices) if response else [],
    }


def wait_until_service_gone(planner_node, client_node, client):
    """Spin so graph updates arrive, then report whether the client still sees the service."""
    executor = SingleThreadedExecutor()
    executor.add_node(planner_node)
    executor.add_node(client_node)
    deadline = time.monotonic() + GRAPH_SETTLE_TIMEOUT_S
    while client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    executor.remove_node(client_node)
    executor.remove_node(planner_node)
    return client.service_is_ready()


def main(shutdown_from_active):
    rclpy.init()
    planner_node = PlannerNode()
    client_node = rclpy.create_node("planner_probe_client")
    client = client_node.create_client(PlanPath, SERVICE_NAME)
    report = {"ready_before_activate": client.service_is_ready()}
    planner_node.trigger_configure()
    planner_node.trigger_activate()
    report["ready_when_active"] = client.wait_for_service(timeout_sec=CALL_TIMEOUT_S)
    report["round_trip"] = call_plan_path(planner_node, client_node, client)
    if shutdown_from_active:
        planner_node.trigger_shutdown()
    else:
        planner_node.trigger_deactivate()
    report["ready_after_exit"] = wait_until_service_gone(planner_node, client_node, client)
    if not shutdown_from_active:
        planner_node.trigger_activate()
        report["ready_after_reactivate"] = client.wait_for_service(timeout_sec=CALL_TIMEOUT_S)
        report["round_trip_after_reactivate"] = call_plan_path(planner_node, client_node, client)
    print(json.dumps(report))
    planner_node.destroy_node()
    client_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main(shutdown_from_active=sys.argv[1] == "shutdown")
