# lightnav_controller

`controller_node` (lifecycle) turns the accepted path, `/lightnav/pose`, `/lightnav/obstacle_status` and `/lightnav/local_grid` into `/lightnav/cmd_vel_raw` (`VelocityCommand`) and `/lightnav/follow_status` at 20 Hz while active.

- Follow layer (pure pursuit, priority 2), avoid layer (candidate arcs, priority 1) and `arbitrate` are pure C++ in `lightnav_controller_core`.
- Avoid layer: 15 arcs (`arc_candidate_count`) at `v_avoid`, rejected on an occupied cell or when any sample along the arc is farther than `max_path_deviation_m` from the path (`arc_length_m` defaults to 2.0), lowest score wins. While the stop zone is occupied or `ObstacleStatus` is stale it holds still (`BLOCKED` / `OBSTACLE_STALE`); with no feasible arc it also waits (`BLOCKED`), then reports `STUCK` / `is_stuck` after `stuck_timeout_s`.
- Grid older than `grid_timeout_s` (or never received) means the avoid layer does not assert; safety_gate stops the robot because `ObstacleStatus` goes stale at the same time.
- Near the path end the layer slows with `k_end` and hands over to the follow layer (`AT_PATH_END`) unless the end point is occupied (then BLOCKED, STUCK).
- Grid contract (frame `grid_frame`, unrotated finite origin, otherwise treated as missing): `nav_msgs/OccupancyGrid` in the frame named by the `grid_frame` parameter (default `base_link`), occupied when `data >= 50`; cells outside the grid count as free. A grid with any other `frame_id` is treated as missing, so the avoid layer never acts. `grid_frame` must be the same as `base_frame` of `obstacle_monitor`, which stamps the grid: `docker/config/robot.yaml` sets both to `base_footprint` (the TF comes from the description container), and `lightnav_bringup/test/test_robot_config_frames.py` fails when they differ. Configuration fails when `grid_frame` is empty.

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

Choose the deviation limit for the course width and specify it in the course YAML.
A small deviation limit (1.5 m in the test) causes a safe stop / `STUCK` for an
obstacle in the path center. No difference between 1.0 m and 2.0 m arc lengths
has been confirmed in this test scenario.
The closed-loop tests use a 0.5 m radius obstacle with 0.5 m inflation, a 2.0 m arc,
and a 2.5 m deviation limit; a centered obstacle with a 1.5 m limit must stop safely.
