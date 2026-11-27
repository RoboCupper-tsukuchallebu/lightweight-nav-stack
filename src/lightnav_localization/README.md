# lightnav_localization

LiDAR・車輪オドメトリ・GPSをGTSAMの固定長窓の姿勢グラフで融合し、`map` 座標の `base_footprint`
姿勢を出す自己位置推定パッケージ。lifecycle ノード（`localization_node`）が `mode` で 2 通りに動く。

- `localization`: 事前地図（`map.pcd`）と照合して、`map` 座標の姿勢を出す。
- `mapping`: 地図を作る。`~/save_map`（か deactivate / shutdown）で全キーフレームをバッチ再最適化し、
  `map_output_dir` に `map.pcd` / `map.yaml` / `trajectory.csv` を書く。
  保存は 3 段階で、段階ごとに 3 つのファイルをそろえて置き換える。停止の制限時間で途中で打ち切られても、
  最後に終わった段階の地図が残る（どの段階まで進んだかはログの `map save: ...` と `map.yaml` の `batch_optimized` で分かる）。
  1. 仮の地図: オンライン推定の姿勢のまま書く（最適化を待たない。`batch_optimized: false`、`position_std_m` は `nan`）。
  2. 最適化後: バッチ再最適化の結果で書く（`batch_optimized: true`、`position_std_m` は `nan`）。
  3. 最終: ループ閉じ込み（行う場合）のあと、標準偏差つきで書く（`save_stage: final`）。ループ閉じ込みを行ったときだけ `loop_closure_count: <本数>` も書く。

  | 保存のきっかけ | `loop_closure_enabled: false`（既定） | `loop_closure_enabled: true` |
  |---|---|---|
  | `~/save_map` の呼び出し | 段階 1 → 2 → 3（ループなし） | 段階 1 → 2 → **ループ閉じ込み** → 3 |
  | 停止（deactivate / shutdown。`docker compose stop`） | 段階 1 → 2 → 3（ループなし） | 段階 1 → 2 → 3（**ループなし**） |

  `map.yaml` の `save_stage`（`preliminary` / `optimized` / `final`）と `loop_closure_count` は、無くても読める（古い地図）。
  - 最適化（段階 2）が失敗したら、保存は失敗を返し、段階 1 の地図が残る。ループ閉じ込みか段階 3 が失敗したら、失敗を返し、段階 2 の地図が残る。
  - ループ閉じ込みか段階 3 が失敗したら、その呼び出しで足したループの拘束をすべて外し、姿勢グラフを段階 2 の状態に戻す。`save_map` の失敗では記録を止めないので、そのあとの停止時の保存は、ループなしで段階 3 まで進める。
  - 前の保存が段階 2 以上まで書いていて、キーフレームが増えていなければ、次の保存は段階 1 を書かずに段階 2 から始める（進んだ地図を仮の地図で上書きしない）。
  - `save_map` が成功したあと、キーフレームが増えていなければ、もう一度呼んでも保存し直さずに成功を返す（`message` は `map already saved (no keyframe added since)`）。成功の `message` は `saved <キーフレーム数> keyframes, <点の数> map points to <出力先>; loop closures: <本数 または off>; <秒> s`。
  - `~/save_map` が成功したあとは記録を止め、停止時の保存は飛ばす（ループを閉じた地図が上書きされない）。**屋外では「`save_map` を呼んでから止める ＝ ループを閉じた地図」「呼ばずに止める ＝ ループなしの地図」**。
  - ループ閉じ込みの間（Mac で 3 分強）、ノードはほかの処理をしない（位置も状態も出ない）。**機体を止めてから `save_map` を呼ぶ**。応答は終わるまで返らない。
  - **停止時の保存の制限時間は 45 秒**（`docker/compose.yaml` の `sigterm_timeout:=45`、そのあと `sigkill_timeout:=10`、全体は `stop_grace_period: 60s`）。超えるとノードは強制終了され、最後に終わった段階の地図が残る。ループ閉じ込みの途中で止められたときは段階 2 の地図。
  - ビューアのサービス画面から止めると `save_map` は呼ばれない（`docker compose stop` と同じ扱いで、ループなしの地図になる）。ループを閉じた地図が要るときは、止める前に `save_map` を呼ぶ。

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
| `imu_topic`（既定 `""` = 購読しない。ロボットでは `/imu/data_raw`） | `sensor_msgs/Imu` | best effort, keep last 2000 | 外付けのジャイロ（「IMU（外付けのジャイロ）」の節） |
| `/lightnav/localization/odom` | `nav_msgs/Odometry` | reliable, depth 10 | `map`→`base_footprint`、共分散あり、stamp は入力 `/wheel/odom` と同一 |
| `/lightnav/localization/status` | `lightnav_msgs/LocalizationStatus` | reliable, transient_local, depth 1 | `status_rate_hz`（既定 2 Hz）+ 状態変化時 |
| `/lightnav/localization/aligned_scan` | `sensor_msgs/PointCloud2` | SensorDataQoS | デバッグ（`map` 座標） |
| `/lightnav/localization/map_cloud` | `sensor_msgs/PointCloud2` | reliable, transient_local, depth 1 | Foxglove 用のプレビュー（`map` 座標）。mapping: これまでのスキャンを `visualization_voxel_m` で間引いて積んだ点（オンライン推定の姿勢で置くので、保存される `map.pcd` とは少し違う）。localization: 事前地図。購読者がいて内容が変わったときだけ `visualization_rate_hz` で出す |
| `/lightnav/localization/path` | `nav_msgs/Path` | reliable, transient_local, depth 1 | これまでの `map`→`base_footprint` の軌跡（`path_min_step_m` ごと、最大 `path_max_poses` 個）。`map_cloud` と同じ条件で出す |
| `/tf` `map→odom` | TF | — | `publish_tf: false` で停止 |
| `~/set_initial_pose` | `lightnav_msgs/SetInitialPose` | 既定 | `frame_id=map`。orientation が単位元かつ `covariance[35] < 0` なら yaw 未指定 |
| `~/reload_map` | `std_srvs/Trigger` | 既定 | `map_file` / `map_yaml` を読み直して再初期化 |
| `~/save_map` | `std_srvs/Trigger` | 既定 | mapping モードだけ。3 段階で `map_output_dir` に保存する（`loop_closure_enabled` が真ならループ閉じ込みも行う。詳細は冒頭）。成功するとその後のキーフレームの記録を止める（続きを記録するには mapping を起動し直す）。応答はループ閉じ込みが終わるまで返らない |

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

W-1 は既定で無効。スリップ判定の根拠は LiDAR の照合だけなので、誤照合とスリップを区別できない。有効化には独立したスリップの根拠と検証用の bag が必要。

既定値は `config/localization.yaml`。主なもの:

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `mode` | `localization` | `localization` / `mapping`。それ以外は `on_configure` が失敗 |
| `map_file` / `map_yaml` | `""` | 地図 PCD / 原点 yaml（`map_file` は必須） |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_footprint` | |
| `gps_frame` | `gnss` | GPS アンテナの frame |
| `imu_topic` ほか `imu_*` | `""`（無効）ほか | 外付け IMU。一覧は「IMU（外付けのジャイロ）」の節 |
| `wheel_timeout_s` / `scan_timeout_s` / `gps_timeout_s` | 0.2 / 0.5 / 3.0 | 入力の鮮度 |
| `init_gps_max_std_m` / `init_scan_count` / `init_yaw_step_deg` | 3.0 / 3 / 15 | 初期位置合わせ |
| `init_gps_reported_std_max_m` | 0.0（判定しない） | 地図の原点と初期位置合わせに使う測位の、**受信機が申告する誤差**の上限。申告 ＝ 測位に付いている誤差の、東と北の大きい方の平方根（型が「不明」以外で、値が正で有限のとき）。超える間は原点・初期化を待ち、`last_error` に `waiting for a GPS fix with reported std <= <上限> m (latest <値> m)`（ログは WARN、2 秒に 1 回まで）を出す。使える測位が来たら消える。申告の無い測位は弾かない。走行中の姿勢グラフに入る標準偏差は変えない（状態ごとの表の値） |
| `init_coarse_corr_m` / `init_fine_corr_m` / `init_inlier_ratio_min` | 2.0 / 0.5 / 0.6 | 初期 yaw 照合 |
| `publish_tf` / `status_rate_hz` | true / 2.0 | |
| `wheel_yaw_slip_threshold_rad` / `wheel_sigma_yaw_per_rad` / `wheel_sigma_yaw_per_m` | 0.0 / 0.1 / 0.01 | 既定では無効。閾値を正の値にすると、受理された LiDAR 照合と車輪のキーフレーム間の相対 yaw の差が閾値を超えた場合に限り、車輪 yaw の標準偏差をその差以上に広げる。それ以外は従来の式を使う |
| `wheel_robust_kernel` / `wheel_robust_k` | huber / 8.0 | スリップと判定した車輪因子の白色化残差に適用する Huber または cauchy。地図・GPS の `huber_k` とは独立。σ を不一致の大きさまで広げるため白色化残差はおよそ 1σ 以下になり、カーネルは追加の保険で、現状ほぼ効かない。8.0 は暫定値 |
| `absolute_sigma_tilt_rad` / `absolute_sigma_z_m` | 0.0 / 0.0 | 既定では無効（0）。正の値にすると、すべてのノードに「車体は水平に近い」（`absolute_sigma_tilt_rad`。mapping と localization の両方）「高さは最初のノードに近い」（`absolute_sigma_z_m`。**mapping のときだけ**。localization では正の値でも入れない: 地図との照合で高さが決まり、出発点と違う場所で初期化すると「最初のノードの高さ」がその場所の高さになって、高低差のあるコースで照合を引っぱるため）という弱い絶対の拘束を入れる。負や非有限の値は configure で拒む。LiDAR オドメトリの roll / pitch / z は相対の拘束だけだと積み上がり、GNSS なしの 650 m の走行で高さが ±8 m 流れた（2026-11-25）。小さすぎると坂が潰れる: 0.05 rad では 10° の坂が平らになる。実走で試した値は 0.3 rad / 2.0 m（高さの流れ ±0.3 m。2 m 程度の高低差は残る） |
| `loop_closure_enabled` | false | 真のとき、**`~/save_map` の呼び出しのときだけ**（停止時の保存では行わない）、mapping の保存の段階 2 と 3 の間に、同じ場所へ戻ったキーフレームの組を探してバッチ最適化に入れる。候補は「推定上で近い組」と「見え方が似ている組（推定が大きく外れていても見つかる）」。照合は大域初期化と同じ 2D 探索 + 3D 検証。37 分の走行で数分かかる（Mac で 3 分強）。停止の制限時間（45 秒）内には終わらないので、停止時の保存では行わない |
| `loop_closure_min_path_gap_m` / `loop_closure_query_spacing_m` | 50.0 / 10.0 | 経路に沿ってこれ以上離れた組だけを候補にする / 経路に沿ってこの間隔で照合する |
| `loop_closure_search_radius_m` / `loop_closure_search_radius_per_m` / `loop_closure_max_search_radius_m` | 5.0 / 0.05 / 20.0 | 推定上で近い組を探す半径。基本 + 経路長 × 係数、上限つき |
| `loop_closure_max_rounds` | 3 | 検出と最適化を繰り返す最大回数（1 回目の補正で次の候補が見つかる） |
| `map_save_keyframe_clouds` | false | 保存時に `keyframes.bin`（全キーフレームの点群。30 分で約 100 MB）も書く。`loop_closure_probe` でループ検出を後から試すためのもの |
| `lidar_odom_slip_gate_yaw_rad` | 0.0 | 既定では拡張ゲートは無効。既存ゲートより大きい値を設定すると有効になる。直進中・停止中とスリップを区別する独立の根拠がないため。有効にすると直進中に最大この値まで yaw の誤照合を通す。車輪 yaw レートが閾値未満の場合に既存ゲートとこの値の大きい方を使い、収束・inlier・並進ゲートも必要 |
| `lidar_odom_slip_wheel_yaw_rate_rad_s` / `lidar_odom_slip_wheel_rate_window_s` | 0.05 / 0.05 | 拡張ゲートの車輪 yaw レート閾値（rad/s）と、その計測窓（秒、有限かつ正） |
| `visualization_enabled` | true | `map_cloud` と `path` を出す。false で両方とも作らない |
| `visualization_rate_hz` / `visualization_voxel_m` / `visualization_max_points` | 0.5 / 0.2 / 1000000 | プレビューの出力周期（ノードの時計）、点の間引き、点数の上限（超えた分は積まない） |
| `path_min_step_m` / `path_max_poses` | 0.1 / 20000 | 軌跡に足す最小の移動量、最大の個数（超えたら古い方から捨てる） |
| `window_s` / `huber_k` | 30.0 / 1.0 | 姿勢グラフ |
| `voxel_size_m` / `keyframe_distance_m` / `keyframe_time_s` | 0.5 / 1.0 / 1.0 | LiDAR オドメトリ |
| `map_match_method` | `gicp` | 事前地図照合の方式（`gicp` / `vgicp`） |
| `map_vgicp_voxel_m` | 0.5 | VGICP の細段ボクセル幅（m） |
| `map_vgicp_two_stage` | false | VGICP の粗密 2 段階照合を有効にする。粗段が未収束なら元の予測から細段を開始 |
| `map_vgicp_coarse_voxel_m` / `map_vgicp_coarse_correspondence_m` | 2.0 / 2.0 | 粗段のボクセル幅 / 対応点距離の上限（m） |
| `map_max_correspondence_distance_m` / `map_max_iterations` | 1.0 / 30 | GICP または VGICP 細段の対応点距離上限（m） / 各段の反復回数上限 |
| `map_inlier_ratio_min` / `map_gate_translation_m` / `map_gate_rotation_rad` | 0.5 / 1.0 / 0.2 | 事前地図照合 |
| `gps_std_m_by_status` | `[2.0, 1.0, 0.3]` | GST 無しの FIX / SBAS / GBAS |
| `gps_min_status` | -1 | この `NavSatStatus` 未満の fix は `Localizer` に渡さない（-1 = 全部受ける）。`docker/config/robot.yaml` は 1（Float 以上）。Single（status 0）は初期化・mapping の原点・因子のどれにも使わず、棄却数にも数えない |
| `gps_min_interval_s` | 0.0 | 前回 `Localizer` に渡した fix の `header.stamp` からこの秒数未満の fix を捨てる（0 = 間引かない）。`robot.yaml` は 1.0（受信機の 5 Hz を 1 Hz に） |
| `gps_gate_mahalanobis` / `gps_trusted_std_m` / `gps_absolute_gate_m` | 3.0 / 0.3 / 2.0 | GPS ゲート |
| `correction_slew_mps` / `correction_slew_rps` / `large_correction_threshold_m` | 0.2 / 0.1 / 1.0 | 補正の平滑化 |
| `tracking_window_s` / `lost_position_std_m` | 5.0 / 2.0 | 状態判定 |
| `odom_only_max_distance_m` / `odom_only_max_s` / `drift_rate` | 50 / 60 / 0.02 | ODOM_ONLY の上限。**`mode: mapping` では無効**（この 2 つで LOST にしない。D5） |

VGICP は対象点のボクセルと隣接する 26 ボクセルを探索するため、対応距離を大きくしても探索範囲はボクセル幅で制限される。VGICP のインライア率は距離条件を満たすボクセル対応がある入力点の割合で、GICP の点同士の対応率とは異なる。`map_inlier_ratio_min` は方式ごとに調整する。

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

`gps_topic`（`robot.yaml` は `/fix`。CLAS 用ファームの受信機が出す Float / Fix。Issue #88）の fix は、購読コールバックで次の順に扱う: 最後に受けた fix の
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
- ループ閉じ込みは `~/save_map` のときだけで、誤検出しうる（見え方が似た別の場所を結ぶ。動きが経路長の 0.2 倍を超える見え方の候補は試さない）。動的物体除去・地図更新は v1 の非スコープ。
- 電源を入れてしばらくは受信機の申告する誤差が大きく、`init_gps_reported_std_max_m` が正のときは、その間は地図の原点と初期位置合わせに使われない（Issue #93 の bag では約 3.5 分）。待っている間に走った区間は地図に入らない。
- 申告の無い測位（誤差の型が「不明」）は `init_gps_reported_std_max_m` で弾かれない。ドライバの設定が変わって申告が出なくなると、この判定は黙って働かなくなる。
- `map.yaml` が `batch_optimized: false` の地図（保存が段階 1 で打ち切られた仮の地図）は、localization が読むと警告のログと `last_error` の注記（`map is a preliminary save (not batch optimized)`）を出す。読み込みは拒まない。

## IMU（外付けのジャイロ。Issue #92）

外付けの IMU のジャイロで yaw の変化を補う。車輪の yaw は滑ると大きくずれる（2026-11-25 の記録で 15〜40°）ので、向きの変化は**車輪よりジャイロを優先**する。
`imu_topic` が空（既定）なら、購読も TF の lookup も診断の項目も作らず、IMU が無いときと同じに動く。

> **起動の手順（必須）**: IMU を使うときは、ロボットを**平らな床に置いてから、起動して 3 秒は機体に触れず、動かさない**。
> 手で持ったまま、傾けたまま起動しない。この静止でジャイロのバイアスと上下軸を決めるため、最初の静止が坂や手の上だと上下軸が傾き、yaw の倍率がずれる（10° で約 1.5 %）。

### 動き

- **入力**: `imu_topic`（ロボットでは `/imu/data_raw`、約 966 Hz の生データ）の `sensor_msgs/Imu`。QoS は best effort、keep last 2000（スキャンの処理の間に届いたぶんを落とさないため）。時刻は `header.stamp` だけで判断し、ノードの時計と比べない（走行記録の再生でも同じに動く）。
- **バイアスと上下軸**: 車輪オドメトリが止まっている区間（並進 0.01 m/s 未満・回転 0.02 rad/s 未満）の 0.5 秒あとから `imu_stationary_window_s` の間の、角速度の平均をバイアス、加速度の平均の向き（重力の反対）を上下軸とする。最初のバイアスが決まるまでジャイロは使わない（車輪の yaw のまま）。
  TF（`base_frame`→IMU）は**照合にだけ**使い、重力で決めた上下軸との角度の差を診断に出す（`imu_up_axis_warn_deg` を超えたら警告）。TF が無くても動く。
- **車輪のコールバックの順**: IMU の購読にたまったメッセージを `take()` で空になるまで取り込む → 車輪の姿勢を渡す → 前の車輪のサンプルから今までの yaw の変化を積分 → `Localizer::on_wheel_odom` に渡す。スキャンの処理は車輪のコールバックの中で走るので、先に取り込まないと増分が「無し」になる。
- **整合を確かめてから使う**: 届いているが誤っているジャイロ（符号が逆、別の軸、倍率の誤り）を使わないため、状態を持つ。`unverified`（`on_configure` の直後。使わない）→ 車輪と LiDAR に合えば `active`（車輪の yaw の増分をジャイロで置き換え、キーフレームの間に yaw の回転係数を足す）。
  `active` のあと、車輪と LiDAR がそろってジャイロと違う区間が出ると `suspended`（LiDAR で確かめ直すまで使わない）、累計 3 回で `disabled_inconsistent`（`on_configure` し直すまで使わない）。これらの状態と負けの累計は、`~/reload_map`・`~/set_initial_pose`・LOST の復帰で `Localizer` を作り直しても持ち越し、`on_cleanup`→`on_configure` で `unverified`・0 に戻す。
  使わない間は車輪の yaw のまま（IMU が無いときと同じ動き）。
- **途絶・遅延**: IMU が止まると車輪の yaw のまま（回転係数は足さない）。スキャンは IMU を待たない。戻れば次の車輪のサンプルから使う。状態は `LocalizationStatus` に出さず、`/diagnostics` だけに出す。
- **mapping**: 回転係数は `~/save_map` と停止時のバッチ再最適化にも効く。

### パラメータ

すべて `on_configure` で読む。雑音の 3 つが有限で正、秒数としきい値が有限で正でないと失敗する（`imu_topic` が空でも検査する）。

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `imu_topic` | `""` | 空 = 無効。ロボットでは `/imu/data_raw` |
| `imu_yaw_prediction_enabled` | `true` | 車輪の yaw の増分をジャイロで置き換える。`false` なら回転係数だけ |
| `imu_gyro_noise_density_rad_s_sqrt_hz` | 1.0e-3 | ジャイロの白色雑音（振動のぶん大きく取る） |
| `imu_gyro_bias_sigma_rad_s` | 5.0e-4 | バイアスの推定の残り |
| `imu_gyro_scale_sigma` | 0.01 | 感度の誤差（回転量に比例） |
| `imu_max_gap_s` | 0.05 | サンプルの間がこれより空いた区間は「ジャイロで覆われていない」とする |
| `imu_max_rate_rad_s` | 10.0 | これを超える角速度のサンプルは捨てる |
| `imu_stationary_window_s` | 2.0 | バイアスと上下軸を求める静止区間の長さ（3 秒未満） |
| `imu_bias_max_abs_rad_s` / `imu_bias_max_std_rad_s` | 0.05 / 0.1 | 静止区間を捨てる条件（角速度の平均の大きさ / 標準偏差） |
| `imu_up_axis_warn_deg` | 10.0 | TF と重力の上下軸の差がこれを超えたら警告 |
| `imu_timeout_s` | 0.5 | 車輪の `header.stamp` が IMU の `header.stamp` よりこの秒数を超えて進むと「途絶」 |

`docker/config/robot.yaml` と `localization_indoor.yaml` は変えていない（既定は無効）。使うときは `imu_topic: /imu/data_raw` を足す。

### `/diagnostics` の `localization_node: IMU`

`imu_topic` が空でないときだけ出る。IMU が無くても推定は続くので、ERROR にはしない。WARN になるのは、`state` が `ready` 以外、`consistency` が `inconsistent`・`suspended`・`disabled_inconsistent`、TF との上下軸の差が上限超えのとき。
3 要素の値は `x,y,z` で、無いものは `unavailable`。

| key | 中身 |
|---|---|
| `state` | `waiting_for_data`（まだ 1 つも届かない）/ `waiting_for_stationary`（バイアス未定）/ `ready` / `stale`（途絶） |
| `consistency` | `unverified` / `inconsistent`（`unverified` のまま「外れ」が 3 回以上）/ `active` / `suspended` / `disabled_inconsistent` |
| `consistency_confirmed_total` / `consistency_outvoted_total` / `consistency_rejected_total` | 「通る」「多数決で負け」「外れ」の回数（負けは `on_configure` からの累計） |
| `last_yaw_deltas_rad` / `verification_sums_rad` | 最後のキーフレーム区間の車輪・ジャイロ・LiDAR の yaw の変化 / 確かめの区間の和 |
| `sample_rate_hz` / `last_sample_age_s` | 直近 1 秒の頻度 / 車輪の最後の時刻 − IMU の最後の時刻 |
| `gyro_bias_rad_s` / `yaw_bias_rad_s` / `bias_age_s` / `bias_updates_total` / `bias_rejected_total` | バイアス（3 軸）、上下軸の成分、更新からの秒数、採用・捨てた静止区間 |
| `up_axis_in_imu` / `tf_up_axis_disagreement_deg` | 重力から決めた上向きの単位ベクトル（IMU の座標）。TF との差（TF が無ければ `unavailable`） |
| `yaw_rate_rad_s` | 最新の、バイアスを引いた yaw の角速度 |
| `yaw_increments_gyro_total` / `yaw_increments_wheel_total` | `active` の間に、増分をジャイロで渡した / 「無し」だった車輪サンプルの数 |
| `rotation_factors_added_total` / `rotation_factors_skipped_total` | 足した回転係数 / 覆いが 95 % に届かず省いた回数 |
| `slip_detected_total` / `slip_max_rad` | 車輪とジャイロの yaw の差が 0.05 rad を超えたキーフレームの数 / 差の最大 |
| `dropped_samples_total` | 捨てたサンプル（有限でない値、角速度の上限、時刻の逆行） |

`Localizer` 側の累計は、`Localizer` を作り直すと 0 に戻る（`consistency_outvoted_total` だけは戻さない）。

### 整合の検査で止まったときの見方

- **`unverified` のまま `inconsistent`（WARN）**: 旋回してもジャイロが車輪・LiDAR と合わない。`verification_sums_rad`（W, G, L）と `last_yaw_deltas_rad` を見る。G の符号が W・L と逆なら取り付けの上下軸か、別のトピックを渡している。G が約 0 なら別の軸。倍率が合わないなら感度の誤り。
- **`up_axis_in_imu` が (0, 0, ±1) から大きく外れている**: 起動時の静止が傾いていた。TF の差（`tf_up_axis_disagreement_deg`）が 10° を超えるなら、URDF の IMU の roll・pitch が実機と違う。`up_axis_in_imu` がそのまま URDF に入れる値の手がかりになる。
- **`suspended` / `disabled_inconsistent`**: 車輪と LiDAR がそろってジャイロと違った。`consistency_outvoted_total` と `last_yaw_deltas_rad` を記録して、数の決まりを見直す材料にする。`disabled_inconsistent` は `on_configure` し直すまで戻らない。
- **`stale`**: IMU が止まった。ケーブルとドライバを見る。この間は車輪の yaw のまま。

### 分かっている限界

- `on_configure` のあと最初の旋回では、車輪が合わせて 0.2 rad 回ったあとの最初のキーフレーム（旋回の始まりから 1.5 秒以内）までは車輪の yaw のまま。そこで回転係数が入り、出力は毎秒 0.1 rad で追いつく。確かめが通るまで、直進が続いても車輪の yaw の偏りはそのまま入る。
- **LiDAR の照合が無いまま確かめが通ったときは、倍率の誤ったジャイロも通りうる**（車輪の報告との比が 0.6〜1.2 倍のとき）。通ったあとも LiDAR の照合が受け入れられなければ見つからず、mapping では曲がった地図がそのまま保存される。単位の誤り、符号の反転、別の軸は通らない。
- ゆっくりした旋回（1 区間あたりの差が 0.05 rad に届かない）での倍率の誤りと、止まらない長い走行でのバイアスのずれは、LiDAR との差が 1 区間で 0.05 rad を超えるまで見つからない。
- 「多数決で負け」が累計 3 回になると、`on_configure` し直すまで車輪の yaw に戻る（今と同じ動き）。
- `active` の途中でジャイロの符号だけが変わった場合、内部のオドメトリが大きくずれて LiDAR の照合が受け入れられず、「負け」が数えられない（LiDAR の値が無いと負けにならないため）。最初から反転したものは使わない。
- 上下軸は最初の静止で決めて、その起動の間使う。静止は車輪で判定するので、手で持って傾けたまま落ち着いた区間も採用されうる。

### 次の段階: 完全な preintegration（`CombinedImuFactor`）が大きな変更になる理由

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

ジャイロの yaw（上の実装）を実機で測った結果、これだけでは足りない（坂の roll・pitch、速度の推定が要る）と分かった場合にだけ、この節の方式を別の Issue として起こす。

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
