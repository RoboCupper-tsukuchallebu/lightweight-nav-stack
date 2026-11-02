"""Lifecycle node serving lightnav_msgs/srv/PlanPath. Holds no planning state."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

from lightnav_msgs.srv import PlanPath
from lightnav_planner.interpolation import (
    DEFAULT_MAX_PATH_POINTS,
    PlanningError,
    plan_straight_path,
)

SERVICE_NAME = "/lightnav/planner/plan_path"


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.declare_parameter("resolution_m", 0.2)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("max_path_points", DEFAULT_MAX_PATH_POINTS)
        self._service = None
        self._resolution_m = 0.0
        self._global_frame = ""
        self._max_path_points = DEFAULT_MAX_PATH_POINTS

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        resolution_m = self.get_parameter("resolution_m").value
        if not (math.isfinite(resolution_m) and resolution_m > 0.0):
            self.get_logger().error(f"invalid resolution_m: {resolution_m}")
            return TransitionCallbackReturn.FAILURE
        global_frame = self.get_parameter("global_frame").value
        max_path_points = self.get_parameter("max_path_points").value
        if not global_frame or max_path_points <= 0:
            self.get_logger().error("global_frame must be non-empty and max_path_points positive")
            return TransitionCallbackReturn.FAILURE
        self._resolution_m = resolution_m
        self._global_frame = global_frame
        self._max_path_points = max_path_points
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._service = self.create_service(PlanPath, SERVICE_NAME, self._handle_plan_path)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._destroy_service()
        return super().on_deactivate(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._destroy_service()
        return TransitionCallbackReturn.SUCCESS

    def _destroy_service(self) -> None:
        if self._service is not None:
            self.destroy_service(self._service)
            self._service = None

    def _handle_plan_path(self, request, response):
        response.success = False
        frame_mismatch = request.frame_id != self._global_frame or (
            request.start.header.frame_id != request.frame_id
        )
        if frame_mismatch:
            response.message = (
                f"frame_id mismatch: expected '{self._global_frame}', request "
                f"'{request.frame_id}', start '{request.start.header.frame_id}'"
            )
            return response
        start_position = request.start.pose.position
        try:
            planned = plan_straight_path(
                (start_position.x, start_position.y),
                [(waypoint.x, waypoint.y) for waypoint in request.waypoints],
                self._resolution_m,
                self._max_path_points,
            )
        except PlanningError as error:
            response.message = str(error)
            return response

        response.path = self._build_path(request.frame_id, planned)
        response.waypoint_indices = list(planned.waypoint_indices)
        response.success = True
        response.message = ""
        return response

    def _build_path(self, frame_id, planned) -> Path:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(planned.points, planned.yaws):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
