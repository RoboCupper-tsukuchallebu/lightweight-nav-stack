# lightnav_sim

`/cmd_vel` を 50 Hz で一輪車モデルに積分し、閉ループ試験用に次を出す純運動学シミュレータ（`kinematic_sim_node`）。

- `nav_msgs/Odometry`（`odom_topic`、既定 `/sim/odom`、`frame_id=world`）: `base_link` から `sensor_offset_xyz` ずらした点を基準にした姿勢。
- 合成 `PointCloud2`（`/velodyne_points`、`lidar_3d`、32 リング）と `LaserScan`（`/scan`、`laser`、`scan_topic` を空にすると無効）。障害物（円・壁）と傾斜地面（`ground_slope_deg`、前方 x 方向に上る）を光線で採取する。
- サービス: `/lightnav/sim/set_pose_publishing`（false で Odometry 停止）、`/lightnav/sim/set_motion_enabled`（false で `/cmd_vel` を無視して姿勢を動かさない）、`/lightnav/sim/clear_obstacles`。

障害物はパラメータ `obstacles`（`[x, y, radius, ...]`）と `walls`（`[x1, y1, x2, y2, ...]`）の平坦な double 配列で渡す。

積分とレイキャストは `lightnav_sim/sim_math.py`（純 Python、pytest 済み）。
