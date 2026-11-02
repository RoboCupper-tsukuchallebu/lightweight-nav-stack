import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
from lightnav_planner.planner_node import PlannerNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from lightnav_msgs.msg import Waypoint
from lightnav_msgs.srv import PlanPath

PROCESSING_TIME_LIMIT_S = 1.0  # measured ~5 ms; loose enough for CI jitter


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def node():
    planner_node = PlannerNode()
    yield planner_node
    planner_node.destroy_node()


def make_request(waypoints, frame_id="map", start_frame_id="map", start_x=0.0):
    request = PlanPath.Request()
    request.frame_id = frame_id
    request.start.header.frame_id = start_frame_id
    request.start.pose.position.x = start_x
    request.waypoints = [Waypoint(x=x, y=y) for x, y in waypoints]
    return request


def plan(node, request):
    return node._handle_plan_path(request, PlanPath.Response())


def configure(node, **parameters):
    node.set_parameters([Parameter(name, value=value) for name, value in parameters.items()])
    return node.trigger_configure()


def test_success_builds_path_with_orientation(node):
    configure(node)
    response = plan(node, make_request([(0.0, 1.0)]))
    assert response.success
    assert response.path.header.frame_id == "map"
    assert len(response.path.poses) == 6
    assert list(response.waypoint_indices) == [5]
    orientation = response.path.poses[0].pose.orientation
    assert orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_request_frame_mismatch_fails(node):
    configure(node)
    response = plan(node, make_request([(1.0, 0.0)], frame_id="odom", start_frame_id="odom"))
    assert not response.success and "frame_id" in response.message


def test_start_frame_mismatch_or_empty_fails(node):
    configure(node)
    assert not plan(node, make_request([(1.0, 0.0)], start_frame_id="odom")).success
    assert not plan(node, make_request([(1.0, 0.0)], start_frame_id="")).success


def test_empty_waypoints_and_non_finite_fail_with_message(node):
    configure(node)
    assert plan(node, make_request([])).message
    assert not plan(node, make_request([(math.nan, 0.0)])).success


def test_oversized_request_fails_without_exception(node):
    configure(node)
    assert not plan(node, make_request([(1e12, 0.0)])).success
    assert not plan(node, make_request([(1e308, 0.0)], start_x=-1e308)).success


@pytest.mark.parametrize(
    "parameters",
    [{"resolution_m": 0.0}, {"resolution_m": math.nan}, {"global_frame": ""}, {"max_path_points": 0}],
)
def test_configure_fails_on_invalid_parameters(node, parameters):
    result = configure(node, **parameters)
    assert result == TransitionCallbackReturn.FAILURE or "ERROR" in str(result)
    assert node._service is None


PROBE_SCRIPT = Path(__file__).with_name("real_service_probe.py")
PROBE_TIMEOUT_S = 60.0


def build_child_environment():
    """macOS SIP strips DYLD_LIBRARY_PATH through colcon's shell, so rebuild it for the child.

    Harmless on Linux, so both CI platforms run the identical test.
    """
    environment = dict(os.environ)
    library_directories = [
        os.path.join(prefix, "lib")
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]
    existing = environment.get("DYLD_LIBRARY_PATH", "")
    environment["DYLD_LIBRARY_PATH"] = os.pathsep.join(
        library_directories + ([existing] if existing else [])
    )
    return environment


def run_real_service_probe(exit_mode):
    completed = subprocess.run(
        [sys.executable, str(PROBE_SCRIPT), exit_mode],
        check=False,
        env=build_child_environment(),
        capture_output=True,
        text=True,
        timeout=PROBE_TIMEOUT_S,
    )
    assert completed.returncode == 0, completed.stderr
    return json.loads(completed.stdout.strip().splitlines()[-1])


@pytest.mark.parametrize("exit_mode", ["deactivate", "shutdown"])
def test_real_service_exists_only_while_active(exit_mode):
    report = run_real_service_probe(exit_mode)
    assert report["service_after_configure"] is False
    assert report["service_after_activate"] is True
    assert report["advertised_when_active"] is True
    assert report["round_trip"] == {"answered": True, "success": True, "waypoint_indices": [5]}
    assert report["service_after_exit_from_active"] is False


def test_processing_time_including_path_assembly(node):
    configure(node)
    waypoints = [(float(index), 0.0) for index in range(1, 101)]
    request = make_request(waypoints)
    began = time.perf_counter()
    response = plan(node, request)
    elapsed_s = time.perf_counter() - began
    print(f"100 waypoints / 100 m / {len(response.path.poses)} poses: {elapsed_s * 1000:.2f} ms")
    assert response.success
    assert elapsed_s < PROCESSING_TIME_LIMIT_S
