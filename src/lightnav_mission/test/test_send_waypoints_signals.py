"""SIGINT の待機制御を ROS 通信なしで検証する。"""

import signal
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from lightnav_mission import send_waypoints


def test_sigint_sets_flag_and_restores_previous_handler(monkeypatch, capsys):
    previous_handler = signal.getsignal(signal.SIGINT)

    def interrupt_run(arguments, interrupted):
        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        assert interrupted.requested
        return send_waypoints.EXIT_OK

    monkeypatch.setattr(send_waypoints, "_run", interrupt_run)
    try:
        assert send_waypoints.main([]) == send_waypoints.EXIT_FAILED
        assert signal.getsignal(signal.SIGINT) is previous_handler
        assert "Ctrl-C" in capsys.readouterr().err
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def test_previous_handler_restored_after_error(monkeypatch):
    previous_handler = signal.getsignal(signal.SIGINT)
    monkeypatch.setattr(send_waypoints, "_run", Mock(side_effect=RuntimeError("failure")))
    try:
        with pytest.raises(RuntimeError, match="failure"):
            send_waypoints.main([])
        assert signal.getsignal(signal.SIGINT) is previous_handler
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def test_spin_stops_after_interrupt(monkeypatch):
    interrupted = SimpleNamespace(requested=False)
    future = Mock()
    future.done.return_value = False
    spin = Mock(side_effect=lambda *arguments, **keywords: setattr(interrupted, "requested", True))
    monkeypatch.setattr(send_waypoints, "rclpy", SimpleNamespace(spin_until_future_complete=spin))
    send_waypoints._spin_until_done(None, None, future, interrupted=interrupted)
    spin.assert_called_once()
    assert spin.call_args.kwargs["timeout_sec"] == send_waypoints.SPIN_SLICE_S


def test_spin_stops_at_deadline(monkeypatch):
    future = Mock()
    future.done.return_value = False
    monkeypatch.setattr(
        send_waypoints, "time", SimpleNamespace(monotonic=Mock(side_effect=[0.0, 0.95, 1.0])))
    spin = Mock()
    monkeypatch.setattr(send_waypoints, "rclpy", SimpleNamespace(spin_until_future_complete=spin))
    send_waypoints._spin_until_done(None, None, future, timeout_s=1.0)
    spin.assert_called_once()
    assert spin.call_args.kwargs["timeout_sec"] == pytest.approx(0.05)


def test_cancel_rejection_warns_without_claiming_success(capsys):
    cancel_future = Mock()
    cancel_future.done.return_value = True
    cancel_future.result.return_value = SimpleNamespace(goals_canceling=[])
    goal_handle = Mock()
    goal_handle.cancel_goal_async.return_value = cancel_future
    send_waypoints._cancel_goal_after_interrupt(None, None, goal_handle)
    message = capsys.readouterr().err
    assert "走行中の可能性" in message
    assert "goal を cancel した" not in message


def test_acceptance_timeout_warns(monkeypatch, capsys):
    send_future = Mock()
    send_future.done.return_value = False
    monkeypatch.setattr(
        send_waypoints, "time", SimpleNamespace(monotonic=Mock(side_effect=[0.0, 5.0])))
    assert send_waypoints._handle_interrupt_before_acceptance(
        None, None, send_future) == send_waypoints.EXIT_FAILED
    send_future.result.assert_not_called()
    assert "走行中の可能性" in capsys.readouterr().err


def test_interrupt_before_send_does_not_send_goal(monkeypatch):
    interrupted = SimpleNamespace(requested=False)
    monkeypatch.setattr(send_waypoints, "load_waypoints_file", Mock(return_value=[
        SimpleNamespace(x=1.0, y=0.0, arrival_radius_m=0.5)]))
    monkeypatch.setattr(
        send_waypoints, "rclpy", SimpleNamespace(Context=Mock(), init=Mock(), create_node=Mock()))
    monkeypatch.setattr(send_waypoints, "SingleThreadedExecutor", Mock())
    monkeypatch.setattr(send_waypoints, "Waypoint", Mock())
    monkeypatch.setattr(send_waypoints, "FollowWaypoints", Mock())
    client = Mock()

    def server_ready():
        interrupted.requested = True
        return True

    client.server_is_ready.side_effect = server_ready
    monkeypatch.setattr(send_waypoints, "ActionClient", Mock(return_value=client))
    assert send_waypoints._run(["unused.yaml"], interrupted) == send_waypoints.EXIT_FAILED
    client.send_goal_async.assert_not_called()


@pytest.mark.parametrize("accepted", [True, False])
def test_interrupt_after_acceptance_cancels_only_accepted_goal(accepted, monkeypatch, capsys):
    goal_handle = Mock(accepted=accepted)
    send_future = Mock()
    send_future.done.return_value = True
    send_future.result.return_value = goal_handle
    cancel_goal = Mock()
    monkeypatch.setattr(send_waypoints, "_cancel_goal_after_interrupt", cancel_goal)
    assert send_waypoints._handle_interrupt_before_acceptance(
        None, None, send_future) == send_waypoints.EXIT_FAILED
    if accepted:
        cancel_goal.assert_called_once_with(None, None, goal_handle)
    else:
        cancel_goal.assert_not_called()
        assert "reject" in capsys.readouterr().err


def test_cancel_timeout_warns(monkeypatch, capsys):
    cancel_future = Mock()
    cancel_future.done.return_value = False
    goal_handle = Mock()
    goal_handle.cancel_goal_async.return_value = cancel_future
    monkeypatch.setattr(
        send_waypoints, "time", SimpleNamespace(monotonic=Mock(side_effect=[0.0, 2.0])))
    send_waypoints._cancel_goal_after_interrupt(None, None, goal_handle)
    message = capsys.readouterr().err
    assert "走行中の可能性" in message
    assert "goal を cancel した" not in message
