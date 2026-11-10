"""Lifecycle starter waits for slow transitions and reports failures."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from lightnav_bringup import lifecycle_start
from lifecycle_msgs.msg import State


def make_node():
    node = MagicMock()
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value={"target": "example", "trace": False}[name])
    change_client = MagicMock()
    state_client = MagicMock()
    node.create_client.side_effect = [change_client, state_client]
    return node, change_client, state_client


def test_slow_configure_waits_for_response(monkeypatch):
    node, change_client, _ = make_node()
    change_client.wait_for_service.return_value = True
    future = MagicMock()
    future.done.return_value = True
    future.exception.return_value = None
    future.result.return_value.success = True
    change_client.call_async.return_value = future
    spins = []
    monkeypatch.setattr(lifecycle_start, "Node", lambda name: node)
    monkeypatch.setattr(lifecycle_start.rclpy, "init", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "spin_until_future_complete",
                        lambda node, future, timeout_sec: spins.append(timeout_sec))
    lifecycle_start.main()
    assert spins == [1.0, 1.0]
    assert change_client.call_async.call_count == 2
    node.destroy_node.assert_called_once()


def test_missing_service_exits_nonzero(monkeypatch):
    node, change_client, _ = make_node()
    change_client.wait_for_service.return_value = False
    monkeypatch.setattr(lifecycle_start, "Node", lambda name: node)
    monkeypatch.setattr(lifecycle_start.rclpy, "init", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "try_shutdown", lambda: None)
    with pytest.raises(SystemExit) as error:
        lifecycle_start.main()
    assert error.value.code == 1
    node.destroy_node.assert_called_once()


def test_unfinished_transition_exits_nonzero(monkeypatch):
    node, change_client, _ = make_node()
    change_client.wait_for_service.return_value = True
    future = MagicMock()
    future.done.return_value = False
    change_client.call_async.return_value = future
    monkeypatch.setattr(lifecycle_start, "Node", lambda name: node)
    monkeypatch.setattr(lifecycle_start.rclpy, "init", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "ok", lambda: False)
    monkeypatch.setattr(lifecycle_start.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None)
    with pytest.raises(SystemExit) as error:
        lifecycle_start.main()
    assert error.value.code == 1
    node.destroy_node.assert_called_once()


def test_configure_response_lost_but_state_eventually_inactive(monkeypatch):
    node, change_client, state_client = make_node()
    change_client.wait_for_service.return_value = True
    pending = MagicMock()
    pending.done.return_value = False
    activated = MagicMock()
    activated.done.return_value = True
    activated.exception.return_value = None
    activated.result.return_value.success = True
    change_client.call_async.side_effect = [pending, activated]
    def state_future(state):
        result = MagicMock()
        result.done.return_value = True
        result.exception.return_value = None
        result.result.return_value.current_state.id = state
        return result
    state_client.call_async.side_effect = [state_future(State.TRANSITION_STATE_CONFIGURING),
                                           state_future(State.PRIMARY_STATE_INACTIVE)]
    monkeypatch.setattr(lifecycle_start, "Node", lambda name: node)
    monkeypatch.setattr(lifecycle_start.rclpy, "init", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(lifecycle_start.rclpy, "ok", lambda: True)
    monkeypatch.setattr(lifecycle_start.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None)
    lifecycle_start.main()
    assert state_client.call_async.call_count == 2
    assert change_client.call_async.call_count == 2
