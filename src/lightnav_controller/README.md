# lightnav_controller

`controller_node` (lifecycle) turns the accepted path, `/lightnav/pose`, `/lightnav/obstacle_status` and `/lightnav/local_grid` into `/lightnav/cmd_vel_raw` (`VelocityCommand`) and `/lightnav/follow_status` at 20 Hz while active.

- Follow layer (pure pursuit, priority 2), avoid layer (candidate arcs, priority 1) and `arbitrate` are pure C++ in `lightnav_controller_core`.
- Avoid layer: 15 arcs (`arc_candidate_count`) at `v_avoid`, rejected on an occupied cell or when any sample along the arc is farther than `max_path_deviation_m` from the path (`arc_length_m` defaults to 2.0), lowest score wins. While the stop zone is occupied or `ObstacleStatus` is stale it holds still (`BLOCKED` / `OBSTACLE_STALE`); with no feasible arc it also waits (`BLOCKED`), then reports `STUCK` / `is_stuck` after `stuck_timeout_s`.
- Grid older than `grid_timeout_s` (or never received) means the avoid layer does not assert; safety_gate stops the robot because `ObstacleStatus` goes stale at the same time.
- Near the path end the layer slows with `k_end` and hands over to the follow layer (`AT_PATH_END`) unless the end point is occupied (then BLOCKED, STUCK).
- Grid contract (frame `base_link`, unrotated finite origin, otherwise treated as missing): `nav_msgs/OccupancyGrid` in `base_link`, occupied when `data >= 50`; cells outside the grid count as free.

Measured (macOS arm64, default build type, gtest `CycleWithFifteenArcsOnEightyByEightyGridIsFast`): one avoid cycle with 15 arcs on an 80x80 grid takes about 0.014 ms (budget 2 ms; CI threshold 10 ms). Follow + arbitrate on a 200 m path takes about 0.0003 ms.

`max_path_deviation_m` has no usable default: each course YAML must specify a finite,
positive value. Configuration fails when it is missing, non-finite, or non-positive.
For example, in a course YAML:

```yaml
controller_node:
  ros__parameters:
    arc_length_m: 2.0
    max_path_deviation_m: 2.5
```

Choose the deviation limit for the course width. Short arcs may not reach the free
space beside an obstacle and can stop the robot. A small deviation limit can prevent
a detour around an obstacle in the path center, causing a safe stop / `STUCK`.
The closed-loop tests use a 0.5 m radius obstacle with 0.5 m inflation, a 2.0 m arc,
and a 2.5 m deviation limit; a centered obstacle with a 1.5 m limit must stop safely.
