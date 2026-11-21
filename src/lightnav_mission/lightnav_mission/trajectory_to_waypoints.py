"""CLI: mapping が保存した trajectory.csv を間引いて waypoints.yaml を書く（Issue #86）。

mapping の手動走行をそのままティーチ走行にする。間引きは record_waypoints と同じ規則。
終了コード: 0 = 書き出した、1 = 入力不正。
"""

import argparse
import os
import sys

from lightnav_mission.cli_arguments import (
    EXIT_INVALID_INPUT,
    EXIT_OK,
    InputErrorArgumentParser,
    is_same_existing_file,
    positive_finite,
)
from lightnav_mission.waypoint_files import (
    WaypointFileError,
    build_waypoints_document,
    load_trajectory_csv,
    samples_to_records,
    select_recorded_samples,
    trajectory_rows_to_samples,
    write_yaml_atomically,
)

RECORDED_WITH_TRAJECTORY = "trajectory"
DEFAULT_SPACING_M = 2.0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = InputErrorArgumentParser(description=__doc__)
    parser.add_argument("trajectory", help="trajectory.csv")
    parser.add_argument("--spacing-m", type=positive_finite, default=DEFAULT_SPACING_M)
    parser.add_argument("--output", default="waypoints.yaml")
    parser.add_argument("--overwrite", action="store_true", help="既存の出力を上書きする")
    return parser


def find_output_problem(arguments) -> str:
    """読み書きの前に出力先を検証する。問題があれば理由、無ければ空文字。"""
    if is_same_existing_file(arguments.output, arguments.trajectory):
        return "出力先が入力と同じ（入力は上書きしない）"
    if os.path.exists(arguments.output) and not arguments.overwrite:
        return f"{arguments.output} が既にある（上書きは --overwrite）"
    return ""


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    output_problem = find_output_problem(arguments)
    if output_problem:
        print(output_problem, file=sys.stderr)
        return EXIT_INVALID_INPUT
    try:
        rows = load_trajectory_csv(arguments.trajectory)
    except WaypointFileError as error:
        print(f"入力不正: {error}", file=sys.stderr)
        return EXIT_INVALID_INPUT
    selected = select_recorded_samples(trajectory_rows_to_samples(rows), arguments.spacing_m)
    document = build_waypoints_document(
        samples_to_records(selected),
        recorded_with=RECORDED_WITH_TRAJECTORY,
        source_trajectory=arguments.trajectory,
    )
    try:
        write_yaml_atomically(arguments.output, document)
    except OSError as error:
        print(f"書き出せない: {error}", file=sys.stderr)
        return EXIT_INVALID_INPUT
    print(f"軌跡 {len(rows)} 行から {len(selected)} 点を {arguments.output} に書き出した")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
