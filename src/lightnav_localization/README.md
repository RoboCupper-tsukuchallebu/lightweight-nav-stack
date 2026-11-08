# lightnav_localization

LiDAR・車輪オドメトリ・GPSをGTSAMの固定長窓の姿勢グラフで融合し、`map` 座標の `base_footprint`
姿勢を出す自己位置推定パッケージ。lifecycle ノード（`localization_node`）が 2 つのモードを持つ。

- `mode: mapping`: 走りながらキーフレームを積み、`~/save_map` で保存時にバッチ再最適化して
  `map.pcd` / `trajectory.csv` / `map.yaml` を書く（L3-b、#30）。
- `mode: localization`（既定）: 保存済みの地図に対して `map` 座標の姿勢を出す（L3-a、#29）。

ロボット（X13）での手順（mapping → `save_map` → localization）は [`docker/README.md`](../../docker/README.md)。

## 起動

```bash
pixi run build
pixi run ros2 launch lightnav_localization localization.launch.py \
  map_file:=/absolute/path/map.pcd \
  map_yaml:=/absolute/path/map.yaml
```

launch は lifecycle を `configure` → `activate` まで自動で進める。launch 引数は
`mode`（`localization` | `mapping`、既定 `localization`）・`map_file`・`map_yaml`・`params_file` の 4 つ。
`map_file` は localization モードでは必須で、読めない・空の場合は `configure` が失敗する
（mapping モードでは要らない）。`map_yaml` は任意で、無い（または原点が無い）場合は GPS を
無効化し `last_error` に残す。`params_file:=...` で `config/localization.yaml` を差し替えられる。
ExecuteProcess の `sigterm_timeout:=` / `sigkill_timeout:=` も `ros2 launch` の引数で渡せる
（Docker の compose は 45 s と 10 s）。

mapping で起動する:

```bash
pixi run ros2 launch lightnav_localization localization.launch.py \
  mode:=mapping params_file:=/absolute/path/params.yaml   # params に map_output_dir が要る
```

ノード単体で動かす場合:

```bash
pixi run ros2 run lightnav_localization localization_node --ros-args \
  -p map_file:=/absolute/path/map.pcd -p map_yaml:=/absolute/path/map.yaml
pixi run ros2 lifecycle set /localization_node configure
pixi run ros2 lifecycle set /localization_node activate
```

## mapping と地図の保存

`mode: mapping` では GNSS・車輪オドメトリ・LiDAR を同じ姿勢グラフに入れてキーフレームを積む
（地図に対する照合は無い）。

1. **原点**: Float 以上の GNSS fix（`init_gps_max_std_m` 以下）が届くと、その位置が `map.yaml` の
   原点になる（INFO ログ `mapping origin set at ...` に緯度経度が出る。公開の場に貼るときは伏せる）。
   原点が決まるまでは `INITIALIZING` のまま `last_error` に出る。
2. **初期 yaw**: 原点が決まってから `init_travel_m`（既定 3 m）を直進すると、その向きが初期 yaw になる。
3. **走行**: キーフレームは `keyframe_distance_m` の移動**または** `keyframe_time_s` の経過で足される。
   静止していても 1 s に 1 個増える。mapping 中は `odom_only_max_distance_m`・`odom_only_max_s`・
   `lost_position_std_m` で LOST にしない。
4. **保存**: `ros2 service call /localization_node/save_map std_srvs/srv/Trigger`。保存時にバッチ再最適化
   （数秒〜十数秒）をして `map_output_dir` へ書く。

| ファイル | 中身 |
|---|---|
| `map.pcd` | 地図点群（`map_voxel_size_m` でボクセル化） |
| `trajectory.csv` | 再最適化後の軌跡（`stamp_sec,x,y,z,roll,pitch,yaw,position_std_m`） |
| `map.yaml` | 原点（緯度経度高度）・作成時刻などのメタデータ。localization が GNSS を使うのに要る |

**書き込みは原子的**: 3 ファイルを同じディレクトリの一時名に書いてから、`map.pcd` → `trajectory.csv` →
`map.yaml` の順に rename する。`map.yaml` を最後にするので、新しい `map.pcd` と古い `map.yaml`（別の GNSS 原点）
の組は残らない。書き込みに失敗したら一時ファイルを消し、前の 3 ファイルを残してエラーを返す。

**保存の契約（#30 の契約変更）**: 旧契約は「`save_map` と終了時（deactivate・shutdown）のどちらでも保存」だった。今は次のとおり。

- `save_map` が成功すると**記録を止める**（`interrupt_mapping()`）。INFO ログ
  `mapping recording stopped after save_map` と status の `last_error` に出る。**これ以後に走った分は地図に入らない。**
  続けたいときは mapping を起動し直す（`map_output_dir` の前の地図は上書きされる）。
- `save_map` の後にキーフレームが増えていなければ、終了時には保存しない（二重のバッチ再最適化と上書きを避ける）。
- `save_map` を呼んでいなければ、終了時に 1 回保存する（LOST 後の部分地図を含む）。
- 終了時の保存が走るには、ノードが SIGINT で終わる必要がある。`ros2 launch` は SIGINT で子に SIGINT を送るが、
  SIGTERM では子を止めない（ノードは強制終了されて保存されない）。Docker の compose は `stop_signal: SIGINT` で渡す
  （[`docker/README.md`](../../docker/README.md)）。

`map_output_dir` を localization が読む `map_file` と同じ場所にすると、mapping を起動するだけで前の地図が
上書きされる。残したい地図は先に退避する。

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
| `/gps/fix`（`gps_topic`） | `sensor_msgs/NavSatFix` | reliable, depth 10 | ENU は `map.yaml` の原点基準。コードの既定は `/gps/fix`、`docker/config/robot.yaml` は `/rtk/fix_all`（mapping と localization で同じ topic） |
| `/lightnav/localization/odom` | `nav_msgs/Odometry` | reliable, depth 10 | `map`→`base_footprint`、共分散あり、stamp は入力 `/wheel/odom` と同一 |
| `/lightnav/localization/status` | `lightnav_msgs/LocalizationStatus` | reliable, transient_local, depth 1 | `status_rate_hz`（既定 2 Hz）+ 状態変化時 |
| `/lightnav/localization/aligned_scan` | `sensor_msgs/PointCloud2` | SensorDataQoS | デバッグ（`map` 座標） |
| `/tf` `map→odom` | TF | — | `publish_tf: false` で停止 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 既定 | 1 Hz、name `localization_node: GNSS`（下の「GNSS の入力と `/diagnostics`」） |
| `~/set_initial_pose` | `lightnav_msgs/SetInitialPose` | 既定 | `frame_id=map`。orientation が単位元かつ `covariance[35] < 0` なら yaw 未指定（CLI 例は下の「初期位置合わせ」） |
| `~/save_map` | `std_srvs/Trigger` | 既定 | mapping モードのみ。成功すると記録を止める（上の「mapping と地図の保存」） |
| `~/reload_map` | `std_srvs/Trigger` | 既定 | `map_file` / `map_yaml` を読み直して再初期化 |

## 初期位置合わせ（localization モード）

1. 位置: `~/set_initial_pose` があればその位置。無ければ GPS fix（`init_gps_max_std_m`
   以下の標準偏差）の ENU。
2. 向き: service に yaw があればそれ。無ければ静止スキャンを `init_scan_count` 個ため、
   `init_yaw_step_deg` 刻みの粗照合 → 精照合を行い、`init_inlier_ratio_min` 以上で採用する。
3. 失敗・TF 待ち・GPS 待ちの間は `INITIALIZING` のまま `last_error` を出す。odom は出さない。

GNSS が無い場所では `set_initial_pose` で位置を与える（`frame_id` は `map`）。

```bash
# 位置と yaw（z = sin(yaw/2)、w = cos(yaw/2)。ここでは yaw = 90°）
ros2 service call /localization_node/set_initial_pose lightnav_msgs/srv/SetInitialPose \
  "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, \
    orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}}}}"
```

yaw を与えず位置だけ決めるときは、orientation を単位元（`w: 1.0`）にして `covariance[35]` を負にする
（`covariance` は 36 要素なので、最後の要素だけ `-1.0` の配列を渡す）。この場合の yaw は静止スキャンで決める。

入力途絶はタイマで `Localizer::poll_state` を呼んで判定する。`/wheel/odom` が
`wheel_timeout_s` 途絶すると odom の出版を止め、status を `INPUT_STALE` にする。

## パラメータ

既定値は `config/localization.yaml`。主なもの:

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `mode` | `localization` | `localization` / `mapping` |
| `map_file` / `map_yaml` | `""` | 地図 PCD / 原点 yaml（`map_file` は localization で必須） |
| `map_output_dir` | `""` | mapping の出力先（mapping で必須。`docker/config/robot.yaml` は `/data/map`） |
| `init_travel_m` / `batch_max_iterations` | 3.0 / 100 | mapping の初期 yaw を決める直進距離 / 保存時のバッチ再最適化の反復上限 |
| `map_voxel_size_m` / `keyframe_cloud_voxel_m` | 0.2 / 0.25 | `map.pcd` のボクセル / キーフレーム点群のボクセル |
| `allow_mapping_without_gps` | false | true なら GNSS 無しで mapping を始められる |
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
| `gps_min_status` | -1 | この `NavSatStatus` 未満の fix は `Localizer` に渡さない（-1 = 全部受ける）。`docker/config/robot.yaml` は 1（Float 以上）。Single（status 0）は初期化・mapping の原点・因子のどれにも使わず、棄却数にも数えない |
| `gps_min_interval_s` | 0.0 | 前回 `Localizer` に渡した fix の `header.stamp` からこの秒数未満の fix を捨てる（0 = 間引かない）。`robot.yaml` は 1.0（受信機の 5 Hz を 1 Hz に） |
| `gps_gate_mahalanobis` / `gps_trusted_std_m` / `gps_absolute_gate_m` | 3.0 / 0.3 / 2.0 | GPS ゲート |
| `correction_slew_mps` / `correction_slew_rps` / `large_correction_threshold_m` | 0.2 / 0.1 / 1.0 | 補正の平滑化 |
| `tracking_window_s` / `lost_position_std_m` | 5.0 / 2.0 | 状態判定 |
| `odom_only_max_distance_m` / `odom_only_max_s` / `drift_rate` | 50 / 60 / 0.02 | ODOM_ONLY の上限。**`mode: mapping` では無効**（この 2 つで LOST にしない。D5） |

## GNSS の入力と `/diagnostics`

`gps_topic`（`robot.yaml` は `/rtk/fix_all`）の fix は、購読コールバックで次の順に扱う: 最後に受けた fix の
`gps_status`・`gps_std_m`（status の表の σ）を記録 → `gps_min_status` 未満なら捨てる → 初期化・mapping の原点 →
`gps_min_interval_s` で間引く → `Localizer::on_gps`（Mahalanobis ゲート）。`LocalizationStatus` の
`gps_status`・`gps_std_m` は、落とす前に記録した**最後に受けた fix** の値（Float から Single に落ちたことが見える）。

`/diagnostics` に 1 Hz で `localization_node: GNSS`（`diagnostic_msgs/DiagnosticArray`）を出す。
`is_gps_conflict` が真のとき WARN、それ以外は OK。

| key | 中身 |
|---|---|
| `gate_rejected_total` | Mahalanobis ゲートで棄却した累計 |
| `time_gap_rejected_total` | 最寄りキーフレームとの時刻差が `gps_keyframe_max_gap_s` を超えて捨てた累計 |
| `conflict_window_rejects` | 直近 `gps_conflict_window` 個のうち、ゲートなら棄却した数（trusted 分岐で受理した分も含む） |
| `accepted_total` | 因子として入れた累計 |
| `gps_status` / `gps_std_m` | 最後に受けた fix の status（-1 は 255）と表の σ |
| `is_gps_conflict` | `true` / `false` |

`gps_min_status` で落とした分と `gps_min_interval_s` で間引いた分は棄却数に入れない（外れ値ではない）。
累計は `Localizer` を作り直す（初期化し直す）と 0 に戻る。

## 既知の限界

- **GPS が無く、地図が進行方向を拘束しない区間では、TRACKING のまま進行方向が車輪の
  スケール誤差どおりにずれる**（Issue #2 の決定 7）。v1 はこれを既知の限界として受け入れ、
  README に明記する（縮退した照合を絶対観測に数えない扱いのため、`position_std_m` を見て
  下流が信頼度を判断する）。
- GPS の高度は拘束しない。坂・段差の roll/pitch は LiDAR オドメトリ由来で、IMU は使わない。
- 初期 yaw の粗照合は 15° 刻み・静止スキャン前提。走行しながらの初期化は対象外。
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
pixi run test    # domain 44, localhost
pixi run lint
```

`test_localization_node` は合成環境（`test/synthetic_world.hpp`）の地図・organized な
NaN 入り点群・`/wheel/odom`・`/gps/fix`・静的 TF を流し、GPS + 未知 yaw からの
`TRACKING`、`set_initial_pose`、入力途絶（G）、出力の契約（N）、地図ファイル無しの
configure 失敗を確認する。`test_initial_alignment` / `test_point_cloud_extraction` が
純ロジックと非有限点の除去を単体で確認する。`test_mapping` は合成コースで mapping（K/K2、停止時保存、
書き込みの原子性、LOST にしないこと）を、`test_localization_launch_*.py`（launch_testing）は
`mode:=localization` / `mode:=mapping` の activate と SIGINT での終了を確認する。
