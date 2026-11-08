# lightnav_localization

LiDAR・車輪オドメトリ・GPSをGTSAMの固定長窓の姿勢グラフで融合し、`map` 座標の `base_footprint`
姿勢を出す自己位置推定パッケージ。Issue #29 の L3 では **localization モード**の lifecycle
ノード（`localization_node`）を実装する。mapping モード（保存時のバッチ再最適化、
`map.pcd` / `map.yaml` / `trajectory.csv`）は L3-b の別 Issue。

## 起動

```bash
pixi run build
pixi run ros2 launch lightnav_localization localization.launch.py \
  map_file:=/absolute/path/map.pcd \
  map_yaml:=/absolute/path/map.yaml
```

launch は lifecycle を `configure` → `activate` まで自動で進める。`map_file` は localization
モードでは必須で、読めない・空の場合は `configure` が失敗する。`map_yaml` は任意で、無い
（または原点が無い）場合は GPS を無効化し `last_error` に残す。`params_file:=...` で
`config/localization.yaml` を差し替えられる。

ノード単体で動かす場合:

```bash
pixi run ros2 run lightnav_localization localization_node --ros-args \
  -p map_file:=/absolute/path/map.pcd -p map_yaml:=/absolute/path/map.yaml
pixi run ros2 lifecycle set /localization_node configure
pixi run ros2 lifecycle set /localization_node activate
```

## フレームと外部パラメータ

TF の木は `map → odom → base_footprint → base_link → velodyne / gnss`。本ノードは
`map→odom` だけを配信し、`odom→base_footprint`（車輪オドメトリ）と
`base_footprint→base_link→velodyne / gnss`（`robot_state_publisher`）は出さない。

LiDAR の外部パラメータ（`base_frame→点群の frame_id`）と GPS アンテナの位置
（`base_frame→gps_frame`、既定 `gnss`）はパラメータではなく **TF から読む**。最初に必要に
なったとき（`on_activate` 後）に lookup し、取れるまでは `INITIALIZING` のまま
`last_error` を出す。`lidar_to_base_xyz_rpy` / `gps_antenna_xyz` パラメータは持たない。

## トピック・サービス

| 名前 | 型 | QoS | 備考 |
|---|---|---|---|
| `/wheel/odom`（`wheel_odom_topic`） | `nav_msgs/Odometry` | reliable, depth 50 | `odom`→`base_footprint`、50 Hz |
| `/velodyne_points`（`points_topic`） | `sensor_msgs/PointCloud2` | reliable, depth 5 | organized / `is_dense: false` / NaN 可。`x,y,z,time` を読み、`time` が無ければ 0、非有限の点は捨てる |
| `/gps/fix`（`gps_topic`） | `sensor_msgs/NavSatFix` | reliable, depth 10 | ENU は `map.yaml` の原点基準 |
| `/lightnav/localization/odom` | `nav_msgs/Odometry` | reliable, depth 10 | `map`→`base_footprint`、共分散あり、stamp は入力 `/wheel/odom` と同一 |
| `/lightnav/localization/status` | `lightnav_msgs/LocalizationStatus` | reliable, transient_local, depth 1 | `status_rate_hz`（既定 2 Hz）+ 状態変化時 |
| `/lightnav/localization/aligned_scan` | `sensor_msgs/PointCloud2` | SensorDataQoS | デバッグ（`map` 座標） |
| `/tf` `map→odom` | TF | — | `publish_tf: false` で停止 |
| `~/set_initial_pose` | `lightnav_msgs/SetInitialPose` | 既定 | `frame_id=map`。orientation が単位元かつ `covariance[35] < 0` なら yaw 未指定 |
| `~/reload_map` | `std_srvs/Trigger` | 既定 | `map_file` / `map_yaml` を読み直して再初期化 |

## 初期位置合わせ（localization モード）

1. 位置: `~/set_initial_pose` があればその位置。無ければ GPS fix（`init_gps_max_std_m`
   以下の標準偏差）の ENU。
2. 向き: service に yaw があればそれ。無ければ静止スキャンを `init_scan_count` 個ため、
   `init_yaw_step_deg` 刻みの粗照合 → 精照合を行い、`init_inlier_ratio_min` 以上で採用する。
3. 失敗・TF 待ち・GPS 待ちの間は `INITIALIZING` のまま `last_error` を出す。odom は出さない。

入力途絶はタイマで `Localizer::poll_state` を呼んで判定する。`/wheel/odom` が
`wheel_timeout_s` 途絶すると odom の出版を止め、status を `INPUT_STALE` にする。

## パラメータ

既定値は `config/localization.yaml`。主なもの:

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `mode` | `localization` | `mapping` は L3-b まで `on_configure` で明示的に失敗 |
| `map_file` / `map_yaml` | `""` | 地図 PCD / 原点 yaml（`map_file` は必須） |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_footprint` | |
| `gps_frame` | `gnss` | GPS アンテナの frame |
| `wheel_timeout_s` / `scan_timeout_s` / `gps_timeout_s` | 0.2 / 0.5 / 3.0 | 入力の鮮度 |
| `init_gps_max_std_m` / `init_scan_count` / `init_yaw_step_deg` | 3.0 / 3 / 15 | 初期位置合わせ |
| `init_coarse_corr_m` / `init_fine_corr_m` / `init_inlier_ratio_min` | 2.0 / 0.5 / 0.6 | 初期 yaw 照合 |
| `publish_tf` / `status_rate_hz` | true / 2.0 | |
| `window_s` / `huber_k` | 30.0 / 1.0 | 姿勢グラフ |
| `voxel_size_m` / `keyframe_distance_m` / `keyframe_time_s` | 0.5 / 1.0 / 1.0 | LiDAR オドメトリ |
| `map_inlier_ratio_min` / `map_gate_translation_m` / `map_gate_rotation_rad` | 0.5 / 1.0 / 0.2 | 事前地図照合 |
| `gps_std_m_by_status` | `[2.0, 1.0, 0.3]` | GST 無しの FIX / SBAS / GBAS |
| `gps_gate_mahalanobis` / `gps_trusted_std_m` / `gps_absolute_gate_m` | 3.0 / 0.3 / 2.0 | GPS ゲート |
| `correction_slew_mps` / `correction_slew_rps` / `large_correction_threshold_m` | 0.2 / 0.1 / 1.0 | 補正の平滑化 |
| `tracking_window_s` / `lost_position_std_m` | 5.0 / 2.0 | 状態判定 |
| `odom_only_max_distance_m` / `odom_only_max_s` / `drift_rate` | 50 / 60 / 0.02 | ODOM_ONLY の上限。**`mode: mapping` では無効**（この 2 つで LOST にしない。D5） |

## 既知の限界

- **GPS が無く、地図が進行方向を拘束しない区間では、TRACKING のまま進行方向が車輪の
  スケール誤差どおりにずれる**（Issue #2 の決定 7）。v1 はこれを既知の限界として受け入れ、
  README に明記する（縮退した照合を絶対観測に数えない扱いのため、`position_std_m` を見て
  下流が信頼度を判断する）。
- GPS の高度は拘束しない。坂・段差の roll/pitch は LiDAR オドメトリ由来で、IMU は使わない。
- 初期 yaw の粗照合は 15° 刻み・静止スキャン前提。走行しながらの初期化は対象外。
- mapping モード、`~/save_map`、`trajectory.csv` は L3-b（別 Issue）。
- mapping モードの K/K2（合成コースの `test_mapping`）の基準は、GPS σ 1.0 m で K（バッチ後の
  `trajectory.csv` の最大誤差）0.45 m、K2（GPS を 100 m 欠測）1.0 m（計測値は pixi で 0.366 m・0.848 m、Docker（apt）で 0.410 m・0.847 m）。
  σ 0.2 m／0.1 m の CLAS 相当では従来どおり 0.2 m 未満。
- mapping 中は LOST にしない（odom_only の 50 m／60 s と、位置の不確かさ `lost_position_std_m` の両方を
  無効にする。`drift_rate` の項が GNSS 欠測の距離で増えるため、後者を残すと約 30 s で LOST になる）。
  localization モードは従来どおり。
- ループ閉じ込み・動的物体除去・地図更新は v1 の非スコープ。

## 試験

```bash
pixi run build
pixi run test    # domain 44, localhost。test_pose_graph の既知 2 件は #24 の判断待ち
pixi run lint
```

`test_localization_node` は合成環境（`test/synthetic_world.hpp`）の地図・organized な
NaN 入り点群・`/wheel/odom`・`/gps/fix`・静的 TF を流し、GPS + 未知 yaw からの
`TRACKING`、`set_initial_pose`、入力途絶（G）、出力の契約（N）、地図ファイル無しの
configure 失敗を確認する。`test_initial_alignment` / `test_point_cloud_extraction` が
純ロジックと非有限点の除去を単体で確認する。
