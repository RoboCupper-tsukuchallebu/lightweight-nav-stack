"""Pure straight-line interpolation. No ROS imports so it is unit-testable."""

import itertools
import math
from collections.abc import Sequence
from dataclasses import dataclass

Point = tuple[float, float]
DEFAULT_MAX_PATH_POINTS = 100_000


@dataclass(frozen=True)
class PlannedPath:
    points: tuple[Point, ...]
    yaws: tuple[float, ...]
    waypoint_indices: tuple[int, ...]


class PlanningError(ValueError):
    """Raised when the request cannot be turned into a path."""


def plan_straight_path(
    start: Point,
    waypoints: Sequence[Point],
    resolution_m: float,
    max_path_points: int = DEFAULT_MAX_PATH_POINTS,
) -> PlannedPath:
    """Connect start and every waypoint with straight lines, sampled every resolution_m.

    waypoint_indices has one entry per input waypoint. A waypoint that repeats the
    previous coordinate shares the index of that point (the duplicate is not emitted).
    """
    if not (math.isfinite(resolution_m) and resolution_m > 0.0):
        raise PlanningError("resolution_m must be positive and finite")
    if len(waypoints) == 0:
        raise PlanningError("waypoints is empty")
    for point in (start, *waypoints):
        if not all(math.isfinite(value) for value in point):
            raise PlanningError("non-finite coordinate")
    _check_point_budget(start, waypoints, resolution_m, max_path_points)

    points = [start]
    waypoint_indices = []
    for waypoint in waypoints:
        previous = points[-1]
        if waypoint != previous:
            points.extend(_sample_segment(previous, waypoint, resolution_m))
        waypoint_indices.append(len(points) - 1)
    return PlannedPath(tuple(points), _headings(points), tuple(waypoint_indices))


def _check_point_budget(
    start: Point, waypoints: Sequence[Point], resolution_m: float, max_path_points: int
) -> None:
    """Reject before allocating: estimate the point count in floating point."""
    estimated_points = 1.0
    previous = start
    for waypoint in waypoints:
        estimated_points += math.dist(previous, waypoint) / resolution_m
        previous = waypoint
    if not math.isfinite(estimated_points) or estimated_points > max_path_points:
        raise PlanningError(
            f"path would need more than max_path_points={max_path_points} points"
        )


def _sample_segment(origin: Point, target: Point, resolution_m: float) -> list[Point]:
    """Points after origin up to and including target; target is always exact."""
    length = math.dist(origin, target)
    step_count = max(1, math.ceil(length / resolution_m))
    return [
        (
            origin[0] + (target[0] - origin[0]) * step / step_count,
            origin[1] + (target[1] - origin[1]) * step / step_count,
        )
        for step in range(1, step_count)
    ] + [target]


def _headings(points: Sequence[Point]) -> tuple[float, ...]:
    """Yaw of each point along the path; the last point reuses the previous heading."""
    if len(points) < 2:
        return (0.0,) * len(points)
    segment_yaws = [
        math.atan2(current[1] - previous[1], current[0] - previous[0]) for previous, current in itertools.pairwise(points)
    ]
    return tuple(segment_yaws + [segment_yaws[-1]])
