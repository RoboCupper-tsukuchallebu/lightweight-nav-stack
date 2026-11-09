"""実行時パラメータの検証とスナップショット。"""

import math

import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from lightnav_mission.mission_node import MissionNode


@pytest.fixture
def mission():
    if not rclpy.ok():
        rclpy.init()
    node = MissionNode()
    yield node
    node.destroy_node()


def set_value(node, name, value):
    return node.set_parameters_atomically([Parameter(name, value=value)])


def test_tuning_changes_both_directions_and_snapshot(mission):
    # Arrange
    before = mission._mission_parameters
    # Act / Assert
    assert set_value(mission, "default_arrival_radius_m", 2.0).successful
    assert before["default_arrival_radius_m"] == 1.0
    assert mission._mission_parameters["default_arrival_radius_m"] == 2.0
    assert set_value(mission, "default_arrival_radius_m", 0.5).successful
    result = mission.set_parameters_atomically([
        Parameter("default_arrival_radius_m", value=2.0),
        Parameter("final_arrival_radius_m", value=0.0),
    ])
    assert not result.successful and result.reason
    assert mission._mission_parameters["default_arrival_radius_m"] == 0.5


@pytest.mark.parametrize("value", [0.6, float("nan"), float("inf"), -float("inf"), 0.0])
def test_safety_rejects_invalid_or_less_conservative(mission, value):
    # Arrange
    startup = mission.get_parameter("pose_timeout_s").value
    # Act
    result = set_value(mission, "pose_timeout_s", value)
    # Assert
    assert startup == 0.5
    assert not result.successful
    assert "startup 0.5" in result.reason and f"requested {value}" in result.reason
    assert mission.get_parameter("pose_timeout_s").value == 0.5


def test_safety_allows_return_to_startup(mission):
    # Arrange
    assert mission.get_parameter("pose_timeout_s").value == 0.5
    # Act / Assert
    assert set_value(mission, "pose_timeout_s", 0.2).successful
    assert set_value(mission, "pose_timeout_s", 0.5).successful


@pytest.mark.parametrize("name,value", [("tick_hz", 20.0), ("pose_topic", "other"),
                                         ("localization_status_topic", "other"),
                                         ("use_sim_time", True)])
def test_structural_parameters_are_rejected(mission, name, value):
    # Arrange
    assert mission.has_parameter(name)
    # Act
    result = set_value(mission, name, value)
    # Assert
    assert not result.successful and result.reason


def test_nonfinite_startup_safety_rejects_all():
    if not rclpy.ok():
        rclpy.init()
    # Arrange
    node = MissionNode(parameter_overrides=[Parameter("pose_timeout_s", value=math.nan)])
    try:
        # Act
        result = set_value(node, "pose_timeout_s", 0.1)
        # Assert
        assert not result.successful and result.reason
    finally:
        node.destroy_node()


def test_applied_and_rejected_changes_are_logged(mission, monkeypatch):
    # Arrange
    messages = []
    logger = mission.get_logger()
    monkeypatch.setattr(logger, "info", lambda message: messages.append(message))
    monkeypatch.setattr(logger, "warn", lambda message: messages.append(message))
    monkeypatch.setattr(mission, "get_logger", lambda: logger)
    # Act
    assert set_value(mission, "pose_timeout_s", 0.2).successful
    assert not set_value(mission, "pose_timeout_s", 0.8).successful
    assert not set_value(mission, "tick_hz", 20.0).successful
    # Assert
    assert any("parameter changed: pose_timeout_s" in message for message in messages)
    assert any("parameter change rejected: pose_timeout_s" in message for message in messages)
    assert any("parameter change rejected: tick_hz" in message for message in messages)


def test_plan_retains_timeout_and_uses_tick_snapshot(mission, monkeypatch):
    # Arrange
    monkeypatch.setattr(mission, "_now_s", lambda: 10.0)
    original = mission._mission_parameters
    pending = mission._begin_plan([], 0, original)
    # Act
    assert set_value(mission, "plan_timeout_s", 9.0).successful
    # Assert
    assert mission._poll_plan(pending, original)[0] == "waiting"
    assert pending.deadline_s == 15.0


@pytest.mark.parametrize("name,value", [
    ("default_arrival_radius_m", float("nan")), ("final_arrival_radius_m", -1.0),
    ("plan_timeout_s", float("inf")), ("v_progress_min", -1.0),
    ("no_motion_timeout_s", 0.0), ("no_motion_distance_m", 0.0),
    ("pass_index_margin", -1),
])
def test_tuning_rejects_invalid_range(mission, name, value):
    # Arrange
    assert mission.has_parameter(name)
    # Act
    result = set_value(mission, name, value)
    # Assert
    assert not result.successful and result.reason


@pytest.mark.parametrize("state", ["unconfigured", "inactive", "active"])
def test_use_sim_time_rejected_in_each_lifecycle_state(mission, state):
    # Arrange
    if state in ("inactive", "active"):
        assert mission.trigger_configure() == TransitionCallbackReturn.SUCCESS
    if state == "active":
        assert mission.trigger_activate() == TransitionCallbackReturn.SUCCESS
    assert mission._state_machine.current_state[1] == state

    # Act
    result = set_value(mission, "use_sim_time", True)

    # Assert
    assert not result.successful
    assert "use_sim_time" in result.reason and "read-only" in result.reason
