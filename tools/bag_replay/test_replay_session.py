"""replay_session.py: merging /tf_static, the node command and the recorded topic names (needs rclpy and tf2_msgs: the replay image or -e eval)."""

import sys
from pathlib import Path

import pytest

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_arguments  # noqa: E402
import replay_session  # noqa: E402


def make_transform(parent, child, x):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = x
    return transform


def test_merges_static_transforms_of_several_publishers():
    first_message = TFMessage(transforms=[make_transform("base_link", "velodyne", 1.0)])
    second_message = TFMessage(transforms=[make_transform("base_link", "imu", 2.0)])

    merged = replay_session.merge_static_transforms([first_message, second_message])

    assert sorted(transform.child_frame_id for transform in merged.transforms) == ["imu", "velodyne"]


def test_keeps_the_newest_transform_of_a_repeated_pair():
    older_message = TFMessage(transforms=[make_transform("base_link", "velodyne", 1.0)])
    newer_message = TFMessage(transforms=[make_transform("base_link", "velodyne", 3.0)])

    merged = replay_session.merge_static_transforms([older_message, newer_message])

    assert [transform.transform.translation.x for transform in merged.transforms] == [3.0]


def test_returns_none_without_messages():
    assert replay_session.merge_static_transforms([]) is None


def make_arguments(tmp_path, imu_topic="", node_params=()):
    argv = ["--bag", str(tmp_path), "--out", str(tmp_path)]
    if imu_topic:
        argv += ["--imu-topic", imu_topic]
    for node_param in node_params:
        argv += ["--node-param", node_param]
    return replay_arguments.parse_arguments("test", argv)


def capture_node_command(monkeypatch, arguments, tmp_path):
    captured = {}
    monkeypatch.setattr(replay_session, "find_node_executable", lambda: "localization_node")
    monkeypatch.setattr(replay_session.subprocess, "Popen", lambda command, **_: captured.setdefault("command", command))
    replay_session.start_node(arguments, tmp_path, None)
    return captured["command"]


def test_the_node_gets_no_imu_topic_without_the_option(monkeypatch, tmp_path):
    command = capture_node_command(monkeypatch, make_arguments(tmp_path), tmp_path)

    assert not any(argument.startswith("imu_topic") for argument in command)


def test_the_node_gets_the_imu_topic_after_the_node_parameters(monkeypatch, tmp_path):
    command = capture_node_command(monkeypatch, make_arguments(tmp_path, "/imu/data_raw", ["x:=1"]), tmp_path)

    assert "imu_topic:=/imu/data_raw" in command
    assert command.index("x:=1") < command.index("imu_topic:=/imu/data_raw")


def test_the_recorded_topic_names_are_read_from_the_bag(tmp_path):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message

    bag_directory = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name="/wheel/odom", type="nav_msgs/msg/Odometry", serialization_format="cdr"))
    writer.write("/wheel/odom", serialize_message(Odometry()), 0)
    del writer  # closes the bag

    assert replay_session.read_recorded_topic_names(bag_directory) == ["/wheel/odom"]


def test_a_missing_bag_directory_is_a_bag_read_error_naming_the_place(tmp_path):
    missing_directory = tmp_path / "no_such_bag"

    with pytest.raises(replay_session.BagReadError, match="no_such_bag.*does not exist"):
        replay_session.read_recorded_topic_names(missing_directory)


def test_a_directory_without_metadata_is_a_bag_read_error(tmp_path):
    with pytest.raises(replay_session.BagReadError, match="metadata"):
        replay_session.read_recorded_topic_names(tmp_path)


def test_a_broken_metadata_yaml_is_a_bag_read_error_with_the_reason(tmp_path):
    (tmp_path / "metadata.yaml").write_text("rosbag2_bagfile_information: [\n")

    with pytest.raises(replay_session.BagReadError, match="yaml"):
        replay_session.read_recorded_topic_names(tmp_path)


def test_opening_the_reader_of_a_broken_bag_is_a_bag_read_error(tmp_path):
    (tmp_path / "metadata.yaml").write_text("rosbag2_bagfile_information: [\n")

    with pytest.raises(replay_session.BagReadError):
        replay_session.open_bag_reader(tmp_path)


def test_main_exits_with_a_message_instead_of_a_traceback_when_the_bag_cannot_be_read(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "47")
    monkeypatch.setattr(sys, "argv", ["replay_session.py", "--bag", str(tmp_path / "no_such_bag"), "--out", str(tmp_path / "out")])

    with pytest.raises(SystemExit) as exit_information:
        replay_session.main()

    assert isinstance(exit_information.value.code, str)  # a message: SystemExit with text exits non-zero
    assert "no_such_bag" in exit_information.value.code
