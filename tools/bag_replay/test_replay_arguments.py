"""replay_arguments.py: argument rules and the interpolated initial pose (no ROS needed)."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_arguments  # noqa: E402


def parse(*argv):
    return replay_arguments.parse_arguments("test", list(argv))


def test_defaults_keep_the_existing_options_working():
    arguments = parse("--extra-topic", "/a", "--extra-topic", "/b", "--node-param", "x:=1")

    assert arguments.extra_topic == ["/a", "/b"]
    assert arguments.node_param == ["x:=1"]
    assert arguments.record_gnss_topic is None
    assert arguments.save_map_service is False
    assert arguments.save_timeout == 600.0


def test_initial_pose_is_three_numbers():
    arguments = parse("--mode", "localization", "--initial-pose", "1.5,-2,0.25")

    assert arguments.initial_pose == (1.5, -2.0, 0.25)


@pytest.mark.parametrize("text", ["1,2", "1,2,3,4", "a,b,c", "1,2,nan", ""])
def test_a_malformed_initial_pose_is_an_error(text):
    with pytest.raises(SystemExit):
        parse("--mode", "localization", "--initial-pose", text)


def test_save_map_service_is_for_mapping_only():
    parse("--save-map-service")
    with pytest.raises(SystemExit):
        parse("--mode", "localization", "--save-map-service")


def test_initial_pose_options_are_for_localization_only_and_exclusive():
    with pytest.raises(SystemExit):
        parse("--initial-pose", "0,0,0")
    with pytest.raises(SystemExit):
        parse("--initial-pose-from-trajectory", "t.csv")
    with pytest.raises(SystemExit):
        parse("--mode", "localization", "--initial-pose", "0,0,0", "--initial-pose-from-trajectory", "t.csv")


def test_save_timeout_must_be_positive():
    assert parse("--save-timeout", "30").save_timeout == 30.0
    with pytest.raises(SystemExit):
        parse("--save-timeout", "0")


def test_played_topics_add_the_gnss_topic_once():
    sensors = ("/velodyne_points", "/tf")

    assert replay_arguments.played_topics(sensors, ["/fix_all"], "/fix_all") == ["/velodyne_points", "/tf", "/fix_all"]
    assert replay_arguments.played_topics(sensors, ["/x"], None) == ["/velodyne_points", "/tf", "/x"]


SENSORS = ("/velodyne_points", "/wheel/odom", "/tf", "/tf_static")


def make_status(name, **values):
    return SimpleNamespace(name=name, values=[SimpleNamespace(key=key, value=value) for key, value in values.items()])


def test_imu_topic_defaults_to_empty_and_gives_the_node_no_parameter():
    arguments = parse()

    assert arguments.imu_topic == ""
    assert replay_arguments.imu_node_parameters(arguments.imu_topic) == []


def test_imu_topic_is_played_and_given_to_the_node():
    arguments = parse("--imu-topic", "/imu/data_raw")

    assert replay_arguments.played_topics(SENSORS, arguments.extra_topic, arguments.record_gnss_topic, arguments.imu_topic) == [
        *SENSORS, "/imu/data_raw"]
    assert replay_arguments.imu_node_parameters(arguments.imu_topic) == ["imu_topic:=/imu/data_raw"]


def test_without_an_imu_topic_the_played_topics_are_the_same_as_before():
    assert replay_arguments.played_topics(SENSORS, ["/fix_all"], None) == [*SENSORS, "/fix_all"]
    assert replay_arguments.played_topics(SENSORS, ["/fix_all"], None, "") == [*SENSORS, "/fix_all"]


def test_an_imu_topic_already_asked_for_as_extra_is_played_once():
    assert replay_arguments.played_topics(SENSORS, ["/imu/data_raw"], None, "/imu/data_raw") == [*SENSORS, "/imu/data_raw"]


def test_a_topic_missing_from_the_bag_is_not_played_and_is_reported():
    played, missing = replay_arguments.split_by_recorded_topics(SENSORS, [], None, "/imu/data_raw", [*SENSORS, "/other"])

    assert played == list(SENSORS)
    assert missing == ["/imu/data_raw"]


def test_a_topic_in_the_bag_is_played_and_not_reported():
    played, missing = replay_arguments.split_by_recorded_topics(SENSORS, [], None, "/imu/data_raw", [*SENSORS, "/imu/data_raw"])

    assert played == [*SENSORS, "/imu/data_raw"]
    assert missing == []


def test_extra_and_gnss_topics_missing_from_the_bag_are_reported_in_request_order():
    played, missing = replay_arguments.split_by_recorded_topics(
        SENSORS, ["/a", "/b"], "/fix_all", "/imu/data_raw", [*SENSORS, "/b", "/fix_all"])

    assert played == [*SENSORS, "/b", "/fix_all"]
    assert missing == ["/a", "/imu/data_raw"]


def test_sensor_topics_are_played_even_when_the_bag_lacks_them():
    played, missing = replay_arguments.split_by_recorded_topics(SENSORS, [], None, "", [])

    assert played == list(SENSORS)
    assert missing == []


def test_nothing_is_reported_missing_without_extra_requests():
    assert replay_arguments.split_by_recorded_topics(SENSORS, [], None, "", list(SENSORS)) == (list(SENSORS), [])


def test_the_imu_diagnostic_values_are_taken_from_the_array():
    statuses = [make_status("other"), make_status("localization_node: IMU", state="ready", sample_rate_hz="966.0")]

    assert replay_arguments.find_diagnostic_values(statuses) == {"state": "ready", "sample_rate_hz": "966.0"}


def test_the_imu_diagnostic_is_none_when_the_array_does_not_hold_it():
    assert replay_arguments.find_diagnostic_values([make_status("localization_node: Wheel", state="ok")]) is None
    assert replay_arguments.find_diagnostic_values([]) is None


def test_the_last_status_with_the_name_wins():
    statuses = [make_status("localization_node: IMU", state="old"), make_status("localization_node: IMU", state="new")]

    assert replay_arguments.find_diagnostic_values(statuses) == {"state": "new"}


ROWS = [(100.0, 0.0, 0.0, 0.0), (101.0, 2.0, 4.0, 1.0), (102.0, 2.0, 4.0, 1.0)]


def test_initial_pose_is_interpolated_from_the_two_rows_around_the_start():
    pose = replay_arguments.interpolate_initial_pose(ROWS, 100.25)

    assert (pose["x"], pose["y"], pose["yaw"]) == pytest.approx((0.5, 1.0, 0.25))
    assert (pose["before_stamp_s"], pose["after_stamp_s"]) == (100.0, 101.0)


def test_initial_pose_yaw_takes_the_short_way_round():
    rows = [(0.0, 0.0, 0.0, math.radians(170.0)), (1.0, 0.0, 0.0, math.radians(-170.0))]

    pose = replay_arguments.interpolate_initial_pose(rows, 0.5)

    assert abs(pose["yaw"]) == pytest.approx(math.pi)


@pytest.mark.parametrize("query_time", [99.9, 102.1])
def test_a_start_outside_the_map_times_is_an_error(query_time):
    with pytest.raises(replay_arguments.InitialPoseError, match="outside"):
        replay_arguments.interpolate_initial_pose(ROWS, query_time)


def test_rows_more_than_two_seconds_apart_are_an_error_but_exactly_two_is_fine():
    assert replay_arguments.interpolate_initial_pose([(0.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)], 1.0)["x"] == pytest.approx(1.0)
    with pytest.raises(replay_arguments.InitialPoseError, match="apart"):
        replay_arguments.interpolate_initial_pose([(0.0, 0.0, 0.0, 0.0), (2.5, 2.0, 0.0, 0.0)], 1.0)


def test_trajectory_rows_are_read_from_the_saved_csv(tmp_path):
    path = tmp_path / "trajectory.csv"
    path.write_text("stamp_sec,x,y,z,roll,pitch,yaw,position_std_m\n1.5,2,3,4,0,0,0.5,nan\n")

    assert replay_arguments.read_trajectory_rows(path) == [(1.5, 2.0, 3.0, 0.5)]
