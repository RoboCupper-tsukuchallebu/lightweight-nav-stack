# lightnav_planner

`planner_node` (lifecycle) serves `lightnav_msgs/srv/PlanPath` at `/lightnav/planner/plan_path`. It is stateless: it joins `start` and each waypoint with straight lines, resamples them every `resolution_m` (default 0.2) and returns a `nav_msgs/Path` plus `waypoint_indices`, the Path index of each waypoint. Each waypoint is hit exactly. The service is available only while the node is `active`. It fails with `success=false` and a `message` when `frame_id` differs from `global_frame` (or from `start.header.frame_id`), when no waypoint is given, or when a coordinate is not finite. Consecutive identical coordinates are emitted once; the duplicated waypoint shares the index of the retained point.

Parameters: `resolution_m` (0.2), `global_frame` (`map`, non-empty), `max_path_points` (100000). A request that would need more points (or overflows) fails with `success=false` before any allocation.

Processing time: the service handler (request validation, interpolation and `nav_msgs/Path` assembly) takes about 3-4 ms for 100 waypoints over 100 m (501 poses), measured on a Mac with `pytest -s` (`test_processing_time_including_path_assembly`). The test asserts an upper bound of 1 s so CI jitter does not cause failures.
