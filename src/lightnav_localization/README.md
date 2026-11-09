# lightnav_localization

LiDAR・車輪オドメトリ・GPSをGTSAMの固定長窓の姿勢グラフで融合し、`map` 座標の `base_footprint`
姿勢を出す自己位置推定パッケージ。lifecycle ノード（`localization_node`）が `mode` で 2 通りに動く。

- `localization`: 事前地図（`map.pcd`）と照合して、`map` 座標の姿勢を出す。
- `mapping`: 地図を作る。`~/save_map`（か deactivate / shutdown）で全キーフレームをバッチ再最適化し、
  `map_output_dir` に `map.pcd` / `map.yaml` / `trajectory.csv` を書く。

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
| `/lightnav/localization/map_cloud` | `sensor_msgs/PointCloud2` | reliable, transient_local, depth 1 | Foxglove 用のプレビュー（`map` 座標）。mapping: これまでのスキャンを `visualization_voxel_m` で間引いて積んだ点（オンライン推定の姿勢で置くので、保存される `map.pcd` とは少し違う）。localization: 事前地図。購読者がいて内容が変わったときだけ `visualization_rate_hz` で出す |
| `/lightnav/localization/path` | `nav_msgs/Path` | reliable, transient_local, depth 1 | これまでの `map`→`base_footprint` の軌跡（`path_min_step_m` ごと、最大 `path_max_poses` 個）。`map_cloud` と同じ条件で出す |
| `/tf` `map→odom` | TF | — | `publish_tf: false` で停止 |
| `~/set_initial_pose` | `lightnav_msgs/SetInitialPose` | 既定 | `frame_id=map`。orientation が単位元かつ `covariance[35] < 0` なら yaw 未指定 |
| `~/reload_map` | `std_srvs/Trigger` | 既定 | `map_file` / `map_yaml` を読み直して再初期化 |
| `~/save_map` | `std_srvs/Trigger` | 既定 | mapping モードだけ。バッチ再最適化して `map_output_dir` に保存する。成功するとその後のキーフレームの記録を止める（続きを記録するには mapping を起動し直す） |

## 初期位置合わせ（localization モード）

1. 位置: `~/set_initial_pose` があればその位置。無ければ GPS fix（`init_gps_max_std_m`
   以下の標準偏差）の ENU。
2. 向き: service に yaw があればそれ。無ければ静止スキャンを `init_scan_count` 個ため、
   `init_yaw_step_deg` 刻みの粗照合 → 精照合を行い、`init_inlier_ratio_min` 以上で採用する。
3. 失敗・TF 待ち・GPS 待ちの間は `INITIALIZING` のまま `last_error` を出す。odom は出さない。

入力途絶はタイマで `Localizer::poll_state` を呼んで判定する。`/wheel/odom` が
`wheel_timeout_s` 途絶すると odom の出版を止め、status を `INPUT_STALE` にする。

## 室内（GNSS 無し）

GNSS を使わず、LiDAR と車輪だけで mapping と localization を行う（Issue #43 §12）。ノードのコードは屋外と同じで、
params ファイル `docker/config/localization_indoor.yaml` で切り替える
（compose では `.env` の `LIGHTNAV_LOCALIZATION_PARAMS`。compose の 1 行は 1c-3 の後に足す）。

- `gps_topic` を**誰も出していない topic 名**（`/lightnav/gnss_disabled`）にする。`allow_mapping_without_gps: true` だけでは
  GNSS の購読は止まらず、初期化の前に使える fix が 1 つでも届くとそこが GNSS の原点になる。室内の Float（σ 1.0 m）は
  `init_gps_max_std_m` を通る。topic 名を実在のものに間違えると、黙って GNSS に戻る。
- `map.yaml` が `has_gps_origin: false` であれば、localization は GNSS を `gps_topic` に関係なく無効にする。status の
  `last_error` に `map has no GPS origin: GPS disabled` が出るが、知らせのためのもの。
- 状態は mapping 中ずっと `ODOM_ONLY` と出る。事前地図も GNSS も無いためで、正常（mapping 中は LOST にならない）。

手順:

1. 床の目印（テープで十字を描き、車体の前方向も印す）に、車体の中心と向きを合わせて置く。
2. mapping を起動したら、**state が `INITIALIZING` を抜けるまで目印の上で静止する**（activate から `gps_timeout_s`
   3 s の間）。目印の位置が map の原点 (0, 0, 0)、そのときの**車体の前方が +x**、yaw 0 になる。抜けたら**まっすぐ前進して
   走り始める**。パッドでティーチ走行し、`save_map` を呼ぶ。
3. localization を起動し、目印に同じ向きで置いて、`set_initial_pose` で (0, 0, 0)・yaw 0 を与える。`covariance[35]` は 0 のまま
   にする（yaw を指定したことになる。負にすると yaw 未指定で、静止スキャンの照合で yaw を決める）。`TRACKING` になることを確かめる。

   ```bash
   ros2 service call /localization_node/set_initial_pose lightnav_msgs/srv/SetInitialPose \
     "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}}"
   ```

4. 走らせてから目印に戻し、`/lightnav/localization/odom`（`localization_node` の出力）の (x, y, yaw) と (0, 0, 0) の差を記録する
   （評価値。合否は Issue #43 §7-3 の 0.5 m）。`/lightnav/pose` は `lightnav_pose_adapter` が `/lightnav/localization/odom` から
   作る 2D の `PoseStamped` で、`localization_node` 単体の起動では出ない（`robot.launch.py` では出る）。

## パラメータ

既定値は `config/localization.yaml`。主なもの:

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `mode` | `localization` | `localization` / `mapping`。それ以外は `on_configure` が失敗 |
| `map_file` / `map_yaml` | `""` | 地図 PCD / 原点 yaml（`map_file` は必須） |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_footprint` | |
| `gps_frame` | `gnss` | GPS アンテナの frame |
| `wheel_timeout_s` / `scan_timeout_s` / `gps_timeout_s` | 0.2 / 0.5 / 3.0 | 入力の鮮度 |
| `init_gps_max_std_m` / `init_scan_count` / `init_yaw_step_deg` | 3.0 / 3 / 15 | 初期位置合わせ |
| `init_coarse_corr_m` / `init_fine_corr_m` / `init_inlier_ratio_min` | 2.0 / 0.5 / 0.6 | 初期 yaw 照合 |
| `publish_tf` / `status_rate_hz` | true / 2.0 | |
| `wheel_yaw_slip_threshold_rad` / `wheel_sigma_yaw_per_rad` / `wheel_sigma_yaw_per_m` | 0.1 / 0.1 / 0.01 | 受理された LiDAR 照合と車輪のキーフレーム間の相対 yaw の差が閾値を超えた場合に限り、車輪 yaw の標準偏差をその差以上に広げる。それ以外は従来の式を使う |
| `wheel_robust_kernel` / `wheel_robust_k` | huber / 8.0 | スリップと判定した車輪因子の白色化残差に適用する Huber または cauchy。地図・GPS の `huber_k` とは独立。8.0 はスリップ判定時の大きな外れ値に備える保険として、白色化残差 8σ までは二乗損失を保つ暫定値。実データによる検証が必要 |
| `lidar_odom_slip_gate_yaw_rad` | 0.35 | 車輪の直近 0.05 秒の yaw レートの絶対値が `lidar_odom_slip_wheel_yaw_rate_rad_s`（既定 0.05 rad/s）未満の場合に限り、既存ゲートとこの値の大きい方を使う。照合の収束・inlier・並進ゲートは引き続き必要 |
| `visualization_enabled` | true | `map_cloud` と `path` を出す。false で両方とも作らない |
| `visualization_rate_hz` / `visualization_voxel_m` / `visualization_max_points` | 0.5 / 0.2 / 1000000 | プレビューの出力周期（ノードの時計）、点の間引き、点数の上限（超えた分は積まない） |
| `path_min_step_m` / `path_max_poses` | 0.1 / 20000 | 軌跡に足す最小の移動量、最大の個数（超えたら古い方から捨てる） |
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

## 診断とエラーの保持

`/diagnostics` には GNSS に加え、state、map match、errors、performance を同じ配列で 1 Hz に出す。
state は動作状態と位置の不確かさ、map match は一致率と棄却理由別の累計、errors はエラーと出来事の累計、
performance は直近のスキャン処理時間とプロセスの CPU・メモリを示す。CPU は複数コアのため 100 % を超える。
メモリは Linux の RSS で、他の OS では `unavailable` と表示する。

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `diagnostics_cpu_warn_percent` | 200 | CPU 使用率の警告閾値 |
| `diagnostics_memory_warn_mb` | 2048 | RSS の警告閾値 |
| `last_error_hold_s` | 10 | 出来事のエラーを保持する秒数 |

`last_error` の状態エラーは条件が解消すると消える。最適化失敗、地図再構築失敗、車輪時刻の逆行は出来事として扱い、
最後に起きてからノードの時計で `last_error_hold_s` が経過すると消える。累計は `error_events_total` に残る。
これは従来の、再初期化まで出来事を保持する意味からの変更である。GNSS 原点なしと保存後の記録停止の注記は警告扱いにしない。

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
- GPS の高度は拘束しない。坂・段差の roll/pitch は LiDAR オドメトリ由来で、IMU は使わない（足し方は「IMU を後から足すとき」）。
- 初期 yaw の粗照合は 15° 刻み・静止スキャン前提。走行しながらの初期化は対象外。
- mapping モードの K/K2（合成コースの `test_mapping`）の基準は、GPS σ 1.0 m で K（バッチ後の
  `trajectory.csv` の最大誤差）0.45 m、K2（GPS を 100 m 欠測）1.0 m（計測値は pixi で 0.366 m・0.848 m、Docker（apt）で 0.410 m・0.847 m）。
  σ 0.2 m／0.1 m の CLAS 相当では従来どおり 0.2 m 未満。
- mapping 中は LOST にしない（odom_only の 50 m／60 s と、位置の不確かさ `lost_position_std_m` の両方を
  無効にする。`drift_rate` の項が GNSS 欠測の距離で増えるため、後者を残すと約 30 s で LOST になる）。
  localization モードは従来どおり。
- ループ閉じ込み・動的物体除去・地図更新は v1 の非スコープ。

## IMU を後から足すとき

今は IMU を使わない（購読も TF の lookup もパラメータも無い）。Issue #43 の決定で、コードは入れず、足し方だけをここに書く。
行番号は develop の 4d23464 時点のもので、ずれたら関数名で探す。

### 1. 最初の一歩: ジャイロの yaw 係数

車輪の yaw はスケール誤差のぶんだけ曲がる（「既知の限界」の 1 つ目）。ジャイロの z 軸を、続く 2 つのキーフレームの間で積分し、
その間の yaw の変化だけを係数として姿勢グラフに足す。ノードの鍵は今のまま `x` だけで、速度・バイアス・重力は状態に入れない。
加速度計は使わない。まずこれで室内の戻り誤差と屋外の yaw のずれが減るかを測ってから、2 に進むかを決める。

`PoseGraph`（`pose_graph.cpp`）:

- `add_rotation_factor(from_index, to_index, yaw_delta_rad, yaw_sigma_rad)` を足す。中身は Pose3 の `BetweenFactor` で、測定値は
  「並進 0、roll と pitch が 0、yaw だけ `yaw_delta_rad`」。ノイズは `pose_noise`（63 行目）で作り、並進と roll・pitch の sigma は
  大きく（拘束しない値に）して、yaw の sigma だけを効かせる。roll・pitch は車輪の平面拘束（`add_node_with_wheel_factor`、176〜185 行目）
  が見ているので、ここでは触らない。
- 雛形は `add_lidar_odometry_factor`（198〜209 行目）。すでにある 2 つの鍵に係数を 1 本足し、`pending_factors` に積み、
  `record_history` が真なら `history_factors` にも積むだけ。新しい値も stamp も足さないので、`pending_values` / `pending_stamps` /
  `restart_window` / `batch_optimize` は変わらない。mapping の保存時のバッチ再最適化にも、この係数が自動で入る。
- `yaw_sigma_rad = gyro_sigma * sqrt(dt)`。`gyro_sigma` はジャイロの角度ランダムウォーク（rad/√s）、`dt` は 2 つのキーフレームの stamp の差。
  下限は `sigma_floor_rad`（0.005）にそろえ、係数が硬い拘束にならないようにする。バイアスの残りぶん（`bias_sigma * dt`）を二乗和で足すかは、実機の静止データで決める。
- 傾いた地面では、ジャイロの z を base の z とみなす近似が tilt のぶん外れる。平地の室内では問題にならない。

`Localizer`（`localizer.hpp` / `localizer.cpp`）:

- 新しい `imu_buffer.hpp` / `imu_buffer.cpp` に `ImuBuffer` を作る。Eigen だけを使い、ROS に依存しない。サンプル（stamp と、base 系の角速度の z）を持ち、
  `integrate_yaw(from_sec, to_sec)` が台形積分で yaw の変化を返す（両端は補間、範囲外は `nullopt`）。`prune_before` で古いサンプルを捨てる。
  `PoseBuffer`（`pose_buffer.hpp`）と同じ作りにし、保持は `kWheelBufferSec`（3 秒、`localizer.cpp` 19 行目）にそろえる。
  `CMakeLists.txt` の `lightnav_localization_core` のソース一覧（30〜43 行目）に足す。新しい依存は要らない。
- `LocalizerParameters`（`localizer.hpp` 23〜48 行目）に IMU の設定（`gyro_sigma`、`imu_to_base` の回転、静止判定の閾値）を足す。`lidar_to_base` と同じ置き方。
- `Localizer::on_imu(stamp_sec, angular_velocity_in_imu)` を `on_scan` / `on_gps`（`localizer.hpp` 78〜79 行目）の隣に足す。
  `Implementation` の `PoseBuffer buffer`（`localizer.cpp` 106 行目）の隣に `ImuBuffer` を持つ。
- `process_scan`（`localizer.cpp` 321 行目）のキーフレームの分岐で、`add_node_with_wheel_factor` と `add_lidar_odometry_factor`（380〜385 行目）の直後に、
  `last.stamp` から `pending.stamp` までの yaw の変化を `ImuBuffer` から取り、取れたら `add_rotation_factor(last.index, index, ...)` を足す。
  `refresh_after_optimize`（392 行目）より前に足す。最適化に失敗して窓を作り直すと、ノードの番号が振り直されるため。
  取れないとき（IMU の途絶、範囲外）は係数を足さず、今までどおり進める。
- `drain`（452 行目）の `scan_ready` は、車輪が scan の終わりまで届くのを待つ（459 行目）。IMU は待たない方がよい。IMU が止まっても scan が詰まらない（待ち行列は `kMaxPendingScans` = 20 まで）。
  代わりに、係数を省いた回数を数えて `/diagnostics` に出す（GNSS の `gate_rejected_total` と同じ作り）。待つかどうかは仕様として決める。
- バイアスは、静止中のジャイロ z の平均で推定する。静止は、車輪オドメトリの移動がほぼ 0 の区間が一定の秒数続いたとき（`on_wheel_odom` が `last_wheel_pose` との差を見ている）。
  走行中は更新しない。「室内（GNSS 無し）」の手順 2 の、mapping 開始時に `INITIALIZING` を抜けるまで静止する区間が、そのまま初期のバイアス推定に使える。

`LocalizationNode`（`localization_node.cpp`）:

- パラメータ `imu_topic`（既定 `""` = 無効）を足す（`declare_parameters` 141〜224 行目、`read_parameters` 227 行目）。空を許すので、256〜260 行目の
  「入力 topic が空でない」検査には入れない。**空のときは購読を作らず、`Localizer::on_imu` も呼ばない。今の挙動は何も変わらない。**
- `on_configure`（570〜578 行目）で、`imu_topic` が空でなければ `sensor_msgs/Imu` を購読する。QoS は IMU のドライバに合わせる（best effort なら `SensorDataQoS`）。
  `handle_imu` は `handle_wheel_odom`（799 行目）と同じく `is_active()` を確かめ、非有限の値を捨ててから `Localizer::on_imu` を呼ぶ。
- TF は `base_frame → imu` の回転だけを使う。`resolve_gps_extrinsic`（724〜742 行目）と同じ作りで `resolve_imu_extrinsic` を足し、frame は `sensor_msgs/Imu` の `header.frame_id`
  から取る（`resolve_lidar_extrinsic`、699〜722 行目と同じ）。取れるまでは IMU のサンプルを捨て、`last_error` に `waiting for TF base -> imu` を出す。
  角速度を base 系に回してから z 成分を取るので、IMU が傾いて付いていても使える。`reset_runtime_state`（486 行目）で、この取得済みの印を戻す。

足す試験:

- `test_pose_graph.cpp`: 車輪の yaw にスケール誤差を入れ、真の yaw の変化を `add_rotation_factor` に渡すと、係数が無いときより yaw の誤差が小さい。
  並進は係数で動かない。`record_history` が真のとき `batch_optimize` に係数が入る。既存の 5 件は係数を足さない限り変わらない。
- 新しい `test_imu_buffer.cpp`（`CMakeLists.txt` の試験名の一覧、99〜100 行目付近に `imu_buffer` を足す）: 一定の角速度で `ω * dt` になる。両端の補間。範囲外は `nullopt`。`prune_before`。
  `imu_to_base` に 90 度の roll を付けたとき、z 成分が正しく取れる。静止区間のバイアスの推定。
- `test_localizer.cpp`: IMU を入れないときの出力が今と同じ（係数が 0 本）。合成コース（`synthetic_world.hpp`）で車輪の yaw にスケール誤差を入れ、GPS 無しで、IMU がある方が yaw の誤差が小さい。
  IMU が途中で止まっても例外が出ず、出力が続き、省いた回数が増える。
- `test_localization_node.cpp`: `imu_topic` が空なら購読が作られない。指定すると購読ができ、TF が無いと `last_error` に出る。
- 実機: 「室内（GNSS 無し）」の手順 4 の戻り誤差を、IMU 有りと無しで比べる（合否は変えず、評価値として記録する）。

### 2. 完全な preintegration（`CombinedImuFactor`）が大きな変更になる理由

加速度計まで使い、速度とバイアスを状態に入れる方式。精度は上がりうるが、姿勢グラフが「`x` の鎖」である前提が多くの場所に入っているため、`PoseGraph` と `Localizer` の両方を作り直すことになる。

- **鍵が増える**。`key_of`（`pose_graph.cpp` 45 行目）は `'x'` 固定で、速度（`v`）とバイアス（`b`）の鍵が要る。ノード 1 個あたり 3 つの鍵になり、`Marginals`（`batch_optimize` の 300 行目）の計算も重くなる。
- **`pending_stamps` への登録**。`IncrementalFixedLagSmoother` は鍵ごとの stamp で窓から外す。今は `add_first_node`（103〜104 行目）、`restart_window`（135 行目）、`add_node_with_wheel_factor`（188 行目）が `x` の stamp だけを登録している。
  `v` と `b` の stamp も同じ 3 か所で登録しないと、窓から外れない鍵が残るおそれがある（GTSAM の挙動は実装時に確かめる）。
- **`x` 以外の鍵を知らない処理が多い**。`pose` / `position_and_yaw_std`（326〜348 行目）は `x` だけを返す。`optimize` の `history_values` の更新（248〜254 行目）は `history_stamps` を回して `key_of` だけを更新するので、
  `v` と `b` の初期値が作成時のまま残り、バッチ再最適化が遠い初期値から始まる。`BatchPose`（`pose_graph.hpp` の 34〜39 行目）にも速度とバイアスが無い。
- **窓の作り直し**。`restart_window`（119〜160 行目）は、新しいノードの prior と、前のノードへの弱い `BetweenFactor` だけを作る。`v` と `b` の prior と、前の `v` / `b` とのつなぎも要る。
  最適化に失敗した localization の復旧（`localizer.cpp` の `refresh_after_optimize`、185〜196 行目）は `PoseGraph` を作り直して `add_first_node` を呼ぶので、そこでも `v` と `b` を現在の推定から取り直す。
- **`Localizer` が `x` だけを持つ**。`NodeRecord`（43〜47 行目）、`pose_cache`、`lidar_nodes` は姿勢だけ。窓から出るときの後始末（210〜214 行目）に、`v` と `b` の推定値の持ち回りを足す。
- **重力の初期化**。今は `add_first_node` が roll・pitch を 0 ± 0.1 rad の prior（109 行目）で水平とみなしている。`CombinedImuFactor` は重力を引いて加速度を積分するので、傾きの初期値がずれると速度が重力のぶん流れる。
  静止中の加速度から roll・pitch を初期化し、初期速度 0 と初期バイアスの prior を置く。初期化の経路は `set_initial_pose`、GPS、mapping の開始（`maybe_start_mapping`）の 3 つで、すべてに入れる。
- **平面拘束との衝突**。車輪の係数は dz・droll・dpitch = 0 を sigma 0.05 で縛る（`add_node_with_wheel_factor`）。加速度を使うと roll・pitch が観測できるので、坂での重みを見直す。
  `LidarOdometryOverridesPlanarConstraintOnSlope`（`test_pose_graph.cpp`）と同じ種類の調整になる。
- **キーフレーム間の全サンプルが要る**。キーフレームは 1 m か 1 秒ごと（`keyframe_distance_m` / `keyframe_time_s`）で、その間の IMU（100〜200 Hz）をすべて積分器に順に入れる。
  キーフレームの stamp は IMU のサンプルの間に来るので補間する。窓の作り直しや LOST で積分器をリセットする。IMU と他のセンサの時刻のずれも較正する。
- **外部パラメータが回転だけでは足りない**。積分する加速度は IMU の位置のものなので、`base → imu` の並進（レバーアーム）も TF から読む。
- **ノイズの較正**。加速度・ジャイロのノイズ密度とバイアスのランダムウォークを Allan 分散などで求める。値を誤ると、係数が強すぎて地図の照合を押しのけるか、弱すぎて効かない。
- **ビルドの環境**。`CombinedImuFactor` は GTSAM の navigation モジュールにある。pixi と Docker（apt）で GTSAM の版が同じか、リンクできるかを先に確かめる。
- **既存の基準の再計測**。`test_mapping` の K / K2 の基準（0.45 m / 1.0 m）は鍵の追加で変わりうる。値に合わせて基準を動かさず、計測し直して理由を残す。

1 を入れて測った結果、ジャイロの yaw だけでは足りない（坂の roll・pitch、速度の推定が要る）と分かった場合にだけ、2 を別の Issue として起こす。

## 試験

```bash
pixi run build
pixi run test    # domain 44, localhost。test_pose_graph の既知 2 件は #24 の判断待ち
pixi run lint
```

`test_localization_node` は合成環境（`test/synthetic_world.hpp`）の地図・organized な
NaN 入り点群・`/wheel/odom`・`/gps/fix`・静的 TF を流し、GPS + 未知 yaw からの
`TRACKING`、`set_initial_pose`、入力途絶（G）、出力の契約（N）、地図ファイル無しの
configure 失敗に加え、室内（GNSS 無し）の mapping（`gps_topic` を別名にして `/gps/fix` に fix を流しても原点が
GNSS にならない、始点が原点・yaw 0・前進が +x・`has_gps_origin: false`）と、原点の無い地図 + `set_initial_pose`
での `TRACKING` の維持を確認する。`test_initial_alignment` / `test_point_cloud_extraction` が
純ロジックと非有限点の除去を単体で確認する。

bag の再生で mapping するツールは [`tools/bag_replay`](../../tools/bag_replay/README.md)（`replay` Docker ターゲット）。
