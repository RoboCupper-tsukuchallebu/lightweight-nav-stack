"""CLI: /lightnav/pose を購読し、Ctrl-C で間引いたウェイポイントを YAML に書き出す。"""

import argparse
import math
import sys

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from lightnav_mission.waypoint_files import (
    PoseSample,
    build_waypoints_document,
    samples_to_records,
    select_recorded_samples,
)


def yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


class PoseRecorder(Node):
    def __init__(self, pose_topic: str):
        super().__init__("record_waypoints")
        self.samples: list[PoseSample] = []
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

    def _on_pose(self, message: PoseStamped) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = message.pose.position
        sample = PoseSample(
            stamp, position.x, position.y, yaw_from_quaternion(message.pose.orientation)
        )
        if all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw)):
            self.samples.append(sample)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-topic", default="/lightnav/pose")
    parser.add_argument("--spacing-m", type=float, default=2.0)
    parser.add_argument("--output", default="waypoints_raw.yaml")
    arguments = parser.parse_args(argv)

    rclpy.init()
    recorder = PoseRecorder(arguments.pose_topic)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    selected = select_recorded_samples(recorder.samples, arguments.spacing_m)
    recorder.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if not selected:
        print("姿勢を受信できなかったため書き出さない", file=sys.stderr)
        return 1
    document = build_waypoints_document(samples_to_records(selected), recorded_with="online")
    with open(arguments.output, "w", encoding="utf-8") as file:
        yaml.safe_dump(document, file, sort_keys=False)
    print(f"{len(selected)} 点を {arguments.output} に書き出した")
    return 0


if __name__ == "__main__":
    sys.exit(main())
