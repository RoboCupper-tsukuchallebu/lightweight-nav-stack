# lightnav_sim

`/cmd_vel` を 50 Hz で一輪車モデルに積分し、閉ループ試験用に次を出す純運動学シミュレータ（`kinematic_sim_node`）。

- `nav_msgs/Odometry`（`odom_topic`、既定 `/sim/odom`、`frame_id=world`）: `base_link` から `sensor_offset_xyz` ずらした点を基準にした姿勢。
- 合成 `PointCloud2`（`/velodyne_points`、`lidar_3d`、32 リング）と `LaserScan`（`/scan`、`laser`、`scan_topic` を空にすると無効）。障害物（円・壁）と傾斜地面（`ground_slope_deg`、前方 x 方向に上る）を光線で採取する。
- サービス: `/lightnav/sim/set_pose_publishing`（false で Odometry 停止）、`/lightnav/sim/set_motion_enabled`（false で `/cmd_vel` を無視して姿勢を動かさない）、`/lightnav/sim/clear_obstacles`。

障害物はパラメータ `obstacles`（`[x, y, radius, ...]`）と `walls`（`[x1, y1, x2, y2, ...]`）の平坦な double 配列で渡す。

積分とレイキャストは `lightnav_sim/sim_math.py`（純 Python、pytest 済み）。

## TF とマーカー（RViz 用）

- TF（`publish_tf`、既定 true。false で全 TF を止める）: `map`→`world` 恒等（静的）、`world`→`base_link`（ロボット原点、地面上、動的）、`base_link`→`lidar_3d` と `laser`（`sensor_offset_xyz`、静的）。実機・bringup と同時に起動する場合は `publish_tf:=false` にすること。
- `/lightnav/path`（`PathWithId`）を `/lightnav/sim/path_view`（`nav_msgs/Path`、QoS は mission と同じ transient local）へ再配信する（RViz で経路表示用）。
- `/lightnav/sim/markers`（`MarkerArray`、frame `world`）: 円障害物、壁、ロボット箱と向き矢印。毎回 DELETEALL から送るので `clear_obstacles` 後は消える。
- `sensor_offset_xyz` 既定は `[0.1, 0.0, 0.7]`（暫定。LiDAR は地面から約 0.7 m）。

## GUI

`pixi run build` の後、`pixi run -e gui sim-gui`（`launch/sim_gui.launch.py`、シーンは `config/sample_scene.yaml`、RViz は `rviz/sim.rviz`、Fixed Frame は `map`）。`rviz2` は `gui` 環境にだけ入る。
