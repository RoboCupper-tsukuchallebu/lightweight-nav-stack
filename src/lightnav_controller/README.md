# lightnav_controller

`controller_node` (lifecycle) turns the accepted path, `/lightnav/pose`, `/lightnav/obstacle_status` `/lightnav/local_grid` and `/lightnav/local_grid_raw` into `/lightnav/cmd_vel_raw` (`VelocityCommand`) and `/lightnav/follow_status` at 20 Hz while active.

- Follow layer (pure pursuit, priority 2), avoid layer (candidate arcs, priority 1) and `arbitrate` are pure C++ in `lightnav_controller_core`.
- Avoid layer: 15 arcs (`arc_candidate_count`) at `v_avoid`, rejected on an occupied cell or when any sample along the arc is farther than `max_path_deviation_m` from the path (`arc_length_m` defaults to 2.0), lowest score wins. While the stop zone is occupied or `ObstacleStatus` is stale it holds still (`BLOCKED` / `OBSTACLE_STALE`); with no feasible arc it also waits (`BLOCKED`), then reports `STUCK` / `is_stuck` after `stuck_timeout_s`.
- Grid older than `grid_timeout_s` (or never received) means the avoid layer does not assert; safety_gate stops the robot because `ObstacleStatus` goes stale at the same time.
- Near the path end the layer slows with `k_end` and hands over to the follow layer (`AT_PATH_END`) unless the end point is occupied (then BLOCKED, STUCK).
- Grid contract (frame `grid_frame`, unrotated finite origin, otherwise treated as missing): `nav_msgs/OccupancyGrid` in the frame named by the `grid_frame` parameter (default `base_link`), inflated cells occupied when `data >= 50`; raw cells must be binary 0/100; cells outside the grid count as free. A grid with any other `frame_id` is treated as missing, so the avoid layer never acts. `grid_frame` must be the same as `base_frame` of `obstacle_monitor`, which stamps the grid: `docker/config/robot.yaml` sets both to `base_footprint` (the TF comes from the description container), and `lightnav_bringup/test/test_robot_config_frames.py` fails when they differ. Configuration fails when `grid_frame` is empty.

The arc center is checked on the inflated grid; its forward stop-zone rectangle is checked on the raw grid. A missing, malformed, stale, differently stamped or geometrically mismatched raw grid falls back to the inflated grid for the rectangle. The monitor and controller use the stop-zone values forwarded from `robot.yaml`; the monitor performs inflation. The controller does not inflate again.

A Release build on the build machine averaged 1.12 ms over 1,000 candidate evaluations with the configured 0.3 m forward and 0.4 m half-width stop zone, 15 arcs and 0.1 m samples (50 ms cycle budget). This is a mean of pure evaluation, not an end-to-end deadline guarantee.

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
The closed-loop H and footprint tests use a 0.5 m radius obstacle offset 0.3 m, 0.3 m inflation, and a 1.5 m deviation limit.

Runtime parameters are applied between control cycles without resetting the held path or avoidance state. Changes are not saved to YAML and are lost on restart. Configure now rejects configurations violating `v_min <= v_slow <= v_max` or `avoid_trigger_m < avoid_release_m`.

| Class | Parameters |
|---|---|
| Tuning (both directions) | `lookahead_m`, `search_window_m`, `v_max`, `v_slow`, `w_max`, `w_rotate`, `v_avoid`, `k_end`, `goal_tolerance_m`, `rotate_in_place_heading_threshold_rad`, `w_path`, `w_heading`, `w_smooth`, `arc_candidate_count`, `stuck_timeout_s` |
| Safety (at most startup value) | `v_min`, `max_path_deviation_m`, `pose_timeout_s`, `obstacle_status_timeout_s`, `grid_timeout_s`, `arc_step_m`, `rejoin_distance_m` |
| Safety (at least startup value) | `avoid_trigger_m`, `avoid_release_m`, `arc_length_m`, `avoid_release_hold_s` |
| Read-only after construction | `grid_frame`, `control_hz`, `arc_markers_rate_hz`, `use_sim_time` |

All numeric values must be finite and positive; `arc_candidate_count` must be a positive integer. Safety values require a finite startup baseline and may return to that baseline. Set related parameters in an order that preserves the constraints: the `set_parameters` service validates each value separately.

During avoidance, if path progress is less than 0.1 m over `stuck_timeout_s`, the controller stops and enters BLOCKED even when an arc is feasible. It reports STUCK after another `stuck_timeout_s` without progress. When remaining path distance is unavailable, displacement is used instead.

In-place rotation starts when heading error exceeds `rotate_in_place_heading_threshold_rad` (default 1.2 rad), and ends below 80% of that threshold. After `stuck_timeout_s` in BLOCKED, STUCK persists for `stuck_rotate_after_s` (default 2.0 s) before rotation; each cycle re-evaluates arcs. Direction alternates between recovery episodes. `w_rotate` bounds angular speed; the safety gate and drive enable remain authoritative. `FollowStatus` reports the selected layer, STUCK state and commanded linear and angular speeds.

The user decision in Issue #1 (2026-11-09) permits rotation without checking the footprint, obstacles or stop zone and without an angle or attempt limit. People or objects beside or behind the robot can be struck; a person must be ready to use emergency stop. Stale pose or obstacle status still stops the controller.
The existing safety gate still stops all commands when its forward stop zone is occupied. Its acceleration limits can leave a temporary forward output while transitioning from driving to an in-place rotation command.

Avoidance takes priority over heading alignment, including during STUCK rotation. If the stop zone is occupied, the unchanged safety gate stops even an in-place rotation; recovery can therefore remain blocked until the zone clears. A change to that gate requires user confirmation.

### Avoidance arc visualization

`/lightnav/controller/arc_candidates` publishes `visualization_msgs/MarkerArray` in `grid_frame` (on the robot, `base_footprint`). Green lines are feasible candidates, red lines are rejected candidates, and the marker namespace identifies the rejection reason. The selected candidate is thicker; text shows the avoid state. Markers are cleared when avoidance is idle. Publishing requires a subscriber and is capped by the read-only `arc_markers_rate_hz` parameter (default 2.0 Hz; 0 disables it).

`grid_diagnostic_enabled` defaults to `false` and enables grid matching diagnostics for tests only.

When the stop zone is occupied or obstacle information is stale, candidates are not evaluated, so only the state text appears without arcs.
