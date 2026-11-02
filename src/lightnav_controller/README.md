# lightnav_controller

`controller_node` (lifecycle) turns the accepted path, `/lightnav/pose`, `/lightnav/obstacle_status` and `/lightnav/local_grid` into `/lightnav/cmd_vel_raw` (`VelocityCommand`) and `/lightnav/follow_status` at 20 Hz while active.

- Follow layer (pure pursuit, priority 2), avoid layer (candidate arcs, priority 1) and `arbitrate` are pure C++ in `lightnav_controller_core`.
- Avoid layer: 15 arcs (`arc_candidate_count`) at `v_avoid`, rejected on an occupied cell or when the end point is farther than `max_path_deviation_m` from the path, lowest score wins. With no feasible arc it waits (`BLOCKED`), then reports `STUCK` / `is_stuck` after `stuck_timeout_s`.
- Grid older than `grid_timeout_s` (or never received) means the avoid layer does not assert; safety_gate stops the robot because `ObstacleStatus` goes stale at the same time.
- Grid contract: `nav_msgs/OccupancyGrid` in `base_link`, occupied when `data >= 50`; cells outside the grid count as free.

Measured (macOS arm64, default build type, gtest `CycleWithFifteenArcsOnEightyByEightyGridIsFast`): one avoid cycle with 15 arcs on an 80x80 grid takes about 0.014 ms (budget 2 ms; CI threshold 10 ms). Follow + arbitrate on a 200 m path takes about 0.0003 ms.
