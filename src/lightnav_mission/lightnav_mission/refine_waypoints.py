"""CLI: waypoints_raw.yaml を trajectory.csv で補正して waypoints.yaml を書く。

終了コード: 0 = 全点補正、2 = flagged あり、1 = 入力不正。
"""

import argparse
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
    load_waypoints_file,
    refine_waypoints,
    write_yaml_atomically,
)

EXIT_FLAGGED = 2


def build_argument_parser() -> argparse.ArgumentParser:
    parser = InputErrorArgumentParser(description=__doc__)
    parser.add_argument("raw", help="waypoints_raw.yaml")
    parser.add_argument("trajectory", help="trajectory.csv")
    parser.add_argument("--output", default="waypoints.yaml")
    parser.add_argument("--max-correction-m", type=positive_finite, default=2.0)
    parser.add_argument("--max-gap-s", type=positive_finite, default=2.0)
    return parser


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    if any(
        is_same_existing_file(arguments.output, source)
        for source in (arguments.raw, arguments.trajectory)
    ):
        print("出力先が入力と同じ（入力は上書きしない）", file=sys.stderr)
        return EXIT_INVALID_INPUT
    try:
        raw_records = load_waypoints_file(arguments.raw)
        rows = load_trajectory_csv(arguments.trajectory)
    except WaypointFileError as error:
        print(f"入力不正: {error}", file=sys.stderr)
        return EXIT_INVALID_INPUT
    refined = refine_waypoints(
        raw_records, rows, arguments.max_correction_m, arguments.max_gap_s
    )
    document = build_waypoints_document(
        refined,
        recorded_with="refined",
        source_raw=arguments.raw,
        source_trajectory=arguments.trajectory,
    )
    try:
        write_yaml_atomically(arguments.output, document)
    except OSError as error:
        print(f"書き出せない: {error}", file=sys.stderr)
        return EXIT_INVALID_INPUT
    flagged_count = sum(1 for record in refined if record.flagged)
    if flagged_count:
        print(f"{flagged_count} 点が flagged（要確認）", file=sys.stderr)
        return EXIT_FLAGGED
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
