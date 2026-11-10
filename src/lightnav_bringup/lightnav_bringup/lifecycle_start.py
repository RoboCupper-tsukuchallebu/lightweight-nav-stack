"""Start one lifecycle node using the change-state service responses."""
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("lifecycle_start")
    node.declare_parameter("target", "")
    node.declare_parameter("trace", False)
    target = node.get_parameter("target").value
    trace = node.get_parameter("trace").value
    client = node.create_client(ChangeState, f"/{target}/change_state")
    state_client = node.create_client(GetState, f"/{target}/get_state")
    try:
        if not client.wait_for_service(timeout_sec=30.0):
            node.get_logger().error(f"{target}: change_state service unavailable")
            raise SystemExit(1)
        for transition, label in ((Transition.TRANSITION_CONFIGURE, "configure"),
                                  (Transition.TRANSITION_ACTIVATE, "activate")):
            if trace:
                print(f"LIFECYCLE_REQUEST {time.monotonic():.6f} {target} {label}", flush=True)
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if future.done() and future.exception() is None and future.result().success:
                continue
            expected = State.PRIMARY_STATE_INACTIVE if label == "configure" else State.PRIMARY_STATE_ACTIVE
            deadline = time.monotonic() + 120.0
            state_future = None
            while rclpy.ok() and time.monotonic() < deadline:
                if state_future is None:
                    state_future = state_client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, state_future, timeout_sec=0.5)
                if (state_future.done() and state_future.exception() is None and
                        state_future.result().current_state.id == expected):
                    break
                if state_future.done():
                    state_future = None
            else:
                node.get_logger().error(f"{target}: {label} failed or timed out")
                raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
