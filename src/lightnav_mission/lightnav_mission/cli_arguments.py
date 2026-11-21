"""ファイルを変換する CLI（refine_waypoints / trajectory_to_waypoints）で共通の引数処理（ROS 非依存）。"""

import argparse
import math
import os

EXIT_OK = 0
EXIT_INVALID_INPUT = 1


class InputErrorArgumentParser(argparse.ArgumentParser):
    """引数エラーを終了コード 1（入力不正）にする。argparse の既定の 2 は refine_waypoints が flagged に使う。"""

    def error(self, message):
        self.exit(EXIT_INVALID_INPUT, f"{self.prog}: error: {message}\n")


def positive_finite(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("正の有限値でなければならない")
    return value


def is_same_existing_file(first_path: str, second_path: str) -> bool:
    """両方が存在し、同じファイルを指す（symlink を含む）ときだけ真。"""
    return (
        os.path.exists(first_path)
        and os.path.exists(second_path)
        and os.path.samefile(first_path, second_path)
    )
