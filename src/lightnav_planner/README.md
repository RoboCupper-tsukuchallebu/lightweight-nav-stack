# lightnav_planner

`planner_node` (lifecycle) serves `lightnav_msgs/srv/PlanPath` at `/lightnav/planner/plan_path`. It is stateless: it joins `start` and each waypoint with straight lines, resamples them every `resolution_m` (default 0.2) and returns a `nav_msgs/Path` plus `waypoint_indices`, the Path index of each waypoint. Each waypoint is hit exactly. The service is available only while the node is `active`. It fails with `success=false` and a `message` when `frame_id` differs from `global_frame` (or from `start.header.frame_id`), when no waypoint is given, or when a coordinate is not finite. Consecutive identical coordinates are emitted once; the duplicated waypoint shares the index of the retained point.

Parameters: `resolution_m` (0.2), `global_frame` (`map`).

Processing time: see the PR description (measured with `pixi run test`).
