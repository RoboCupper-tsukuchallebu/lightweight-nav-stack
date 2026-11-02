"""CLI: waypoints_raw.yaml を trajectory.csv で補正して waypoints.yaml を書く。

終了コード: 0 = 全点補正、2 = flagged あり、1 = 入力不正。
"""

import argparse
import math
import os
import sys

import yaml

from lightnav_mission.waypoint_files import (
    WaypointFileError,
    build_waypoints_document,
    load_trajectory_csv,
    load_waypoints_file,
    refine_waypoints,
)

EXIT_OK = 0
EXIT_INVALID_INPUT = 1
EXIT_FLAGGED = 2


class InputErrorArgumentParser(argparse.ArgumentParser):
    """引数エラーを終了コード 1（入力不正）にする。2 は flagged 専用。"""

    def error(self, message):
        self.exit(EXIT_INVALID_INPUT, f"{self.prog}: error: {message}\n")


def _positive_finite(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("正の有限値でなければならない")
    return value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = InputErrorArgumentParser(description=__doc__)
    parser.add_argument("raw", help="waypoints_raw.yaml")
    parser.add_argument("trajectory", help="trajectory.csv")
    parser.add_argument("--output", default="waypoints.yaml")
    parser.add_argument("--max-correction-m", type=_positive_finite, default=2.0)
    parser.add_argument("--max-gap-s", type=_positive_finite, default=2.0)
    return parser


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    if any(
        os.path.exists(arguments.output) and os.path.samefile(arguments.output, source)
        for source in (arguments.raw, arguments.trajectory)
        if os.path.exists(source)
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
    with open(arguments.output, "w", encoding="utf-8") as file:
        yaml.safe_dump(document, file, sort_keys=False)
    flagged_count = sum(1 for record in refined if record.flagged)
    if flagged_count:
        print(f"{flagged_count} 点が flagged（要確認）", file=sys.stderr)
        return EXIT_FLAGGED
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
