"""replay_session.py: merging the /tf_static messages of a bag (needs the replay image: rclpy, tf2_msgs)."""

import sys
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))
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
