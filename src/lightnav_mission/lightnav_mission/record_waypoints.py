"""CLI: /lightnav/pose を購読し、Ctrl-C で間引いたウェイポイントを YAML に書き出す。"""

import argparse
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lightnav_mission.waypoint_files import (
    PoseSample,
    build_waypoints_document,
    samples_to_records,
    select_recorded_samples,
    write_yaml_with_fallback,
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


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-topic", default="/lightnav/pose")
    parser.add_argument("--spacing-m", type=float, default=2.0)
    parser.add_argument("--output", default="waypoints_raw.yaml")
    parser.add_argument("--overwrite", action="store_true", help="既存の出力を上書きする")
    return parser


def validate_arguments(arguments) -> str:
    """走行前に検証する。問題があれば理由、無ければ空文字。"""
    if not math.isfinite(arguments.spacing_m) or arguments.spacing_m <= 0.0:
        return "--spacing-m は正の有限値でなければならない"
    if os.path.exists(arguments.output) and not arguments.overwrite:
        return f"{arguments.output} が既にある（上書きは --overwrite）"
    return ""


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    problem = validate_arguments(arguments)
    if problem:
        print(problem, file=sys.stderr)
        return 1

    rclpy.init()
    recorder = PoseRecorder(arguments.pose_topic)
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    selected = select_recorded_samples(recorder.samples, arguments.spacing_m)
    recorder.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if not selected:
        print("姿勢を受信できなかったため書き出さない", file=sys.stderr)
        return 1
    document = build_waypoints_document(samples_to_records(selected), recorded_with="online")
    written_path = write_yaml_with_fallback(arguments.output, document)
    if written_path != arguments.output:
        print(f"{arguments.output} に書けず退避した: {written_path}", file=sys.stderr)
    print(f"{len(selected)} 点を {written_path} に書き出した")
    return 0 if written_path == arguments.output else 1


if __name__ == "__main__":
    sys.exit(main())
