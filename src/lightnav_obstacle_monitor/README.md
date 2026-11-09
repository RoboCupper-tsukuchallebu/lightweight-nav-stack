# lightnav_obstacle_monitor

3D LiDAR (`PointCloud2`) と任意の 2D LiDAR (`LaserScan`、`scan_topic` が空なら無効) から、
車体周りの局所グリッド `/lightnav/local_grid` と `/lightnav/obstacle_status` を出す lifecycle ノード。
仕様は Issue #1 の契約節が正。

- 3D: 点数が `min_points_per_cell` 以上で、セル内の z の最大 − 最小、または近傍との段差が `height_diff_threshold_m` を超えたセルを占有にする。段差は、中心間のユークリッド距離が `step_neighbor_radius_m` 以下の他セルのうち点数条件を満たすセルの最小 z を G とし、対象セルの最大 z − G で判定する。近傍がなければ段差判定では占有にしない。高い側だけを占有にする。既定値では 10 度の一様な坂や 10 cm の縁石は占有にしない。
- 2D: `scan_max_use_range_m` 以内の返りだけ占有。
- stop / slow zone は膨張前のセル数で判定し、`occupied_min_frames` 連続・`clear_hold_s` 保持でヒステリシスをかける。
- 入力途絶・TF 不在は `stale_sources` と `is_input_stale` に出し、10 Hz のウォッチドッグで status を出し続ける。

## 点群の QoS

`points_topic` の購読は `points_qos_reliability` で選ぶ。深さ 5、volatile。`scan_topic`（`LaserScan`）は SensorDataQoS（best-effort）のまま。

| パラメータ | 既定値 | 意味 |
|---|---|---|
| `points_qos_reliability` | `reliable` | 点群購読の信頼性（`reliable` / `best_effort`）。それ以外の値は configure で拒否 |

1.5 MB の `/velodyne_points` を best-effort で受けると rmw_fastrtps の共有メモリ 512 KiB があふれてフレームが欠けるため、既定は reliable（tsukuchalle_drivers #19 が直れば best_effort に戻せる）。

## 3D 占有判定のパラメータ

| パラメータ | 既定値 | 意味 |
|---|---|---|
| `height_diff_threshold_m` | 0.15 | セル内高低差と近傍段差の共通閾値（m、厳密に超えた場合） |
| `min_points_per_cell` | 2 | 対象セル・地面候補セルの最低点数 |
| `step_neighbor_radius_m` | 0.5 | 近傍探索の円の半径（m）。0 で段差判定を無効化。負・非有限値は lifecycle configure 時に拒否 |

## 処理時間の実測

合成点群（32 リング x 2160 方位 = 69,120 点、HDL-32E 相当、地面 + 円柱障害物）1 フレームについて、
TF 適用・投影・占有判定・膨張・zone 集計・`OccupancyGrid` 構築までの 200 回計測
（TF lookup・受信・publish を含まない純ロジック部分、`test_frame_processor`、Release ビルド）。
LiDAR 高さ 0.7 m、80×80 セル、`step_neighbor_radius_m = 0.5`。ctest の今回の出力:

```text
[ MEASURED ] points=69120 median_ms=1.11383 p99_ms=1.29408
```

| 環境 | 中央値 | p99 |
|---|---|---|
| Apple Silicon Mac (osx-arm64) | 1.11383 ms | 1.29408 ms |

目標（中央値 10 ms 未満、p99 20 ms 未満）を満たす。試験の失敗閾値は CI 向けに 100 ms。
ノード経由の受信から publish までの時間は `ObstacleStatus.processing_time_ms` に毎フレーム載る。

## 障害物の記憶

占有セルを `memory_frame`（既定 `odom`）に保存し、現在の車体座標へ戻して stop / slow zone、最近距離、local_grid に加える。再起動・再 configure で記憶は空になる。点群の地面高さ ±0.1 m に `min_points_per_cell` 個以上の点がある非占有セルだけを空きとみなし、センサーから `blind_radius_m` 未満では消さない。点がない方向や 2D スキャンだけでは空きと判断しない。`memory_max_age_s` は観測中のセルに残った古いキーの整理に使う。空きとして観測されたセルはその時点で消し、死角や測線のないセルは期限で消さない。車体から `memory_max_range_m` を超えた記憶は削除する。世界座標のセル中心への丸めにより各軸で最大約半セルの位置誤差が生じる。TF の移動または回転が連続フレーム間の閾値を超えると全記憶を消す。TF が無い場合は入力を stale にし、グリッドを発行しない。TF ジャンプは `obstacle_memory_reset` として一定時間 stale にする。記憶用 TF はセンサー時刻で取得し、未来への外挿時は入力時刻より古く、`tf_max_age_s` 以内の最新 TF を使う。古い場合は stale とする。

| パラメータ | 既定値 | 意味 |
|---|---|---|
| `memory_frame` | `odom` | 固定座標の TF フレーム。実機の TF に合わせる |
| `blind_radius_m` | 0.6 | 空きとして消去しないセンサー周囲の水平半径 |
| `memory_max_age_s` | 30.0 | 観測中のセルに残った古いキーを整理する時間（秒） |
| `memory_max_range_m` | 10.0 | 車体から記憶を保持する最大距離（m） |
| `tf_max_age_s` | 0.5 | 記憶用 TF と入力時刻の最大経過時間（秒） |
| `memory_reset_report_s` | 2.0 | TF ジャンプ後の stale 通知時間（秒） |
| `memory_jump_distance_m` | 1.0 | TF 移動の消去閾値（m） |
| `memory_jump_angle_rad` | 0.785 | TF 回転の消去閾値（rad） |

## 既知の制限

水平な地面・水平搭載を仮定すると、LiDAR 高さ 0.66 m、下向き限界 −30.67° では、地面の点が得られる最短水平距離は `0.66 / tan(30.67°) ≈ 1.12 m`（LiDAR 直下から）となる。近傍段差判定は、半径内に点数条件を満たす地面セルがある場合だけ効く。半径 0.5 m では `0.66 / tan(30.67°) − 0.5 ≈ 0.62 m` より近い上面だけの障害物は、この段差判定では検出できない。これはセル中心と点位置の差を省いた概算であり、実際の境界はグリッドへの割り当てや遮蔽でも変わる。障害物の点自体がない場合も検出できず、近距離の死角は残る。ドライバの min_range 0.9 m により水平約 0.61 m（高さ差 0.66 m の地面に対して）以内の地面の点もない。記憶は一度も観測できなかった障害物や期限切れの障害物を検出できない。

10° の一様な坂で、傾斜方向に中心間距離 0.5 m のセルの両端に点がある場合、高低差は最大 `(0.5 + 0.1) × tan(10°) ≈ 0.106 m`。閾値までの余裕は `0.15 − 0.106 ≈ 0.044 m`（約 4.4 cm）で、測距・姿勢の誤差はこの余裕を減らす。

近傍の最小 z を基準にするので、点数条件を満たす近傍セルの低い外れ値 1 点で半径内のセルが占有になり得る（止まる側の誤り）。

入力が途絶している間は `/lightnav/local_grid` を publish しない（controller の `grid_timeout_s` を発火させるため）。status だけを 10 Hz で出し続ける。
