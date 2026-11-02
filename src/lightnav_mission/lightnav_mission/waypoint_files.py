"""waypoints.yaml / waypoints_raw.yaml / trajectory.csv の読み書きと補正（ROS 非依存）。"""

import bisect
import csv
import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

import yaml

EXPECTED_FRAME_ID = "map"
FLAG_LARGE_CORRECTION = "large_correction"
FLAG_NO_TRAJECTORY = "no_trajectory"
TRAJECTORY_COLUMNS = (
    "stamp_sec", "x", "y", "z", "roll", "pitch", "yaw", "position_std_m",
)


class WaypointFileError(ValueError):
    """入力ファイルが契約に合わないときに投げる。"""


@dataclass(frozen=True)
class WaypointRecord:
    x: float
    y: float
    yaw: float | None = None
    stamp: float | None = None
    arrival_radius_m: float = 0.0
    correction_m: float | None = None
    flagged: str | None = None


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrajectoryRow:
    stamp_sec: float
    x: float
    y: float
    yaw: float


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _finite_number(value, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointFileError(f"{field_name} は数値でなければならない: {value!r}")
    if not math.isfinite(value):
        raise WaypointFileError(f"{field_name} は有限でなければならない: {value!r}")
    return float(value)


def parse_waypoints_document(document) -> list[WaypointRecord]:
    """YAML を読んだ辞書を検証して WaypointRecord の列にする。"""
    if not isinstance(document, dict):
        raise WaypointFileError("トップレベルは辞書でなければならない")
    frame_id = document.get("frame_id", EXPECTED_FRAME_ID)
    if frame_id != EXPECTED_FRAME_ID:
        raise WaypointFileError(f"frame_id は {EXPECTED_FRAME_ID} のみ対応: {frame_id!r}")
    entries = document.get("waypoints")
    if not isinstance(entries, list) or not entries:
        raise WaypointFileError("waypoints は 1 点以上の配列でなければならない")
    records = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise WaypointFileError(f"waypoints[{index}] は辞書でなければならない")
        prefix = f"waypoints[{index}]"
        radius = _finite_number(entry.get("arrival_radius_m", 0.0), prefix + ".arrival_radius_m")
        if radius < 0.0:
            raise WaypointFileError(f"{prefix}.arrival_radius_m は 0 以上")
        optional = {
            name: _finite_number(entry[name], f"{prefix}.{name}")
            for name in ("yaw", "stamp")
            if entry.get(name) is not None
        }
        records.append(WaypointRecord(
            x=_finite_number(entry.get("x"), prefix + ".x"),
            y=_finite_number(entry.get("y"), prefix + ".y"),
            arrival_radius_m=radius,
            correction_m=entry.get("correction_m"),
            flagged=entry.get("flagged"),
            **optional,
        ))
    return records


def load_waypoints_file(path: str) -> list[WaypointRecord]:
    try:
        with open(path, encoding="utf-8") as file:
            document = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as error:
        raise WaypointFileError(f"{path} を読めない: {error}") from error
    return parse_waypoints_document(document)


def waypoint_to_dict(record: WaypointRecord) -> dict:
    entry = {"x": record.x, "y": record.y}
    for name in ("yaw", "stamp", "correction_m", "flagged"):
        value = getattr(record, name)
        if value is not None:
            entry[name] = value
    if record.arrival_radius_m:
        entry["arrival_radius_m"] = record.arrival_radius_m
    return entry


def build_waypoints_document(records: Sequence[WaypointRecord], **metadata) -> dict:
    return {
        "frame_id": EXPECTED_FRAME_ID,
        **metadata,
        "waypoints": [waypoint_to_dict(record) for record in records],
    }


# ---- record_waypoints の間引き ----

def select_recorded_samples(
    samples: Sequence[PoseSample], spacing_m: float
) -> list[PoseSample]:
    """最初の点は即記録、以後 spacing_m 以上離れるたびに記録、最後の点（停止位置）は必ず記録。"""
    if spacing_m <= 0.0:
        raise ValueError("spacing_m は正でなければならない")
    if not samples:
        return []
    recorded = [samples[0]]
    for sample in samples[1:]:
        last = recorded[-1]
        if math.hypot(sample.x - last.x, sample.y - last.y) >= spacing_m:
            recorded.append(sample)
    if recorded[-1] is not samples[-1]:
        recorded.append(samples[-1])
    return recorded


def samples_to_records(samples: Sequence[PoseSample]) -> list[WaypointRecord]:
    return [
        WaypointRecord(x=sample.x, y=sample.y, yaw=sample.yaw, stamp=sample.stamp)
        for sample in samples
    ]


# ---- refine_waypoints の補正 ----

def load_trajectory_csv(path: str) -> list[TrajectoryRow]:
    try:
        with open(path, newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            missing = set(TRAJECTORY_COLUMNS) - set(reader.fieldnames or [])
            if missing:
                raise WaypointFileError(f"trajectory.csv に列が無い: {sorted(missing)}")
            rows = [
                TrajectoryRow(
                    stamp_sec=float(raw["stamp_sec"]),
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    yaw=float(raw["yaw"]),
                )
                for raw in reader
            ]
    except (OSError, ValueError, csv.Error) as error:
        if isinstance(error, WaypointFileError):
            raise
        raise WaypointFileError(f"{path} を読めない: {error}") from error
    if any(not math.isfinite(value) for row in rows for value in (row.stamp_sec, row.x, row.y, row.yaw)):
        raise WaypointFileError("trajectory.csv に非有限値がある")
    return sorted(rows, key=lambda row: row.stamp_sec)


def interpolate_trajectory(
    rows: Sequence[TrajectoryRow], stamp: float, max_gap_s: float
) -> TrajectoryRow | None:
    """stamp を挟む 2 行を線形補間（yaw は角度補間）。挟めない・間隔が広いときは None。"""
    stamps = [row.stamp_sec for row in rows]
    if not rows or stamp < stamps[0] or stamp > stamps[-1]:
        return None
    upper_index = bisect.bisect_left(stamps, stamp)
    upper = rows[upper_index]
    if upper.stamp_sec == stamp:
        return upper
    lower = rows[upper_index - 1]
    gap = upper.stamp_sec - lower.stamp_sec
    if gap > max_gap_s:
        return None
    ratio = (stamp - lower.stamp_sec) / gap
    return TrajectoryRow(
        stamp_sec=stamp,
        x=lower.x + (upper.x - lower.x) * ratio,
        y=lower.y + (upper.y - lower.y) * ratio,
        yaw=wrap_angle(lower.yaw + wrap_angle(upper.yaw - lower.yaw) * ratio),
    )


def refine_waypoint(
    record: WaypointRecord,
    rows: Sequence[TrajectoryRow],
    max_correction_m: float,
    max_gap_s: float,
) -> WaypointRecord:
    if record.stamp is None:
        return replace(record, flagged=FLAG_NO_TRAJECTORY)
    refined_pose = interpolate_trajectory(rows, record.stamp, max_gap_s)
    if refined_pose is None:
        return replace(record, flagged=FLAG_NO_TRAJECTORY)
    correction = math.hypot(refined_pose.x - record.x, refined_pose.y - record.y)
    if correction > max_correction_m:
        return replace(record, correction_m=correction, flagged=FLAG_LARGE_CORRECTION)
    return replace(record, x=refined_pose.x, y=refined_pose.y, correction_m=correction)


def refine_waypoints(
    records: Sequence[WaypointRecord],
    rows: Sequence[TrajectoryRow],
    max_correction_m: float = 2.0,
    max_gap_s: float = 2.0,
) -> list[WaypointRecord]:
    return [refine_waypoint(record, rows, max_correction_m, max_gap_s) for record in records]
