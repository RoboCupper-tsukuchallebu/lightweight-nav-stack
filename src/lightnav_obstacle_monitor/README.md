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
| `step_neighbor_radius_m` | 0.5 | 近傍探索の円の半径（m）。0 で段差判定を無効化。read_only。負・非有限値は lifecycle configure 時に拒否 |

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

## 既知の制限

水平な地面・水平搭載を仮定すると、LiDAR 高さ 0.7 m、下向き限界 −30.67° では、地面の点が得られる最短水平距離は `0.7 / tan(30.67°) ≈ 1.18 m`（LiDAR 直下から）となる。近傍段差判定は、半径内に点数条件を満たす地面セルがある場合だけ効く。半径 0.5 m では `0.7 / tan(30.67°) − 0.5 ≈ 0.68 m` より近い上面だけの障害物は、この段差判定では検出できない。これはセル中心と点位置の差を省いた概算であり、実際の境界はグリッドへの割り当てや遮蔽でも変わる。障害物の点自体がない場合も検出できず、近距離の死角は残る。

10° の一様な坂で、傾斜方向に中心間距離 0.5 m のセルの両端に点がある場合、高低差は最大 `(0.5 + 0.1) × tan(10°) ≈ 0.106 m`。閾値までの余裕は `0.15 − 0.106 ≈ 0.044 m`（約 4.4 cm）で、測距・姿勢の誤差はこの余裕を減らす。

近傍の最小 z を基準にするので、点数条件を満たす近傍セルの低い外れ値 1 点で半径内のセルが占有になり得る（止まる側の誤り）。

入力が途絶している間は `/lightnav/local_grid` を publish しない（controller の `grid_timeout_s` を発火させるため）。status だけを 10 Hz で出し続ける。

実行時パラメータは起動時の値を基準に安全側にのみ変更できます。変更は再起動後には残りません。停止ゾーン ⊂ 減速ゾーン ⊂ グリッドの制約を満たさない設定は configure 時にも拒否します。

## 実行時パラメータ

| パラメータ | 種類 | 保守方向 | 実行時に変更可 |
|---|---|---|---|
| `z_max_m` | safety | ↑ | はい |
| `scan_max_use_range_m` | safety | ↑ | はい |
| `inflation_radius_m` | safety | ↑ | はい |
| `stop_zone_x_m` | safety | ↑ | はい |
| `stop_zone_y_half_m` | safety | ↑ | はい |
| `slow_zone_x_m` | safety | ↑ | はい |
| `slow_zone_y_half_m` | safety | ↑ | はい |
| `clear_hold_s` | safety | ↑ | はい |
| `height_diff_threshold_m` | safety | ↓ | はい |
| `min_points_per_cell` | safety | ↓ | はい |
| `min_cell_count` | safety | ↓ | はい |
| `occupied_min_frames` | safety | ↓ | はい |
| `points_timeout_s` | safety | ↓ | はい |
| `scan_timeout_s` | safety | ↓ | はい |
| `tf_timeout_s` | safety | ↓ | はい |
| `step_neighbor_radius_m` | read_only | なし | いいえ |
| `grid_size_m` | read_only | なし | いいえ |
| `cell_size_m` | read_only | なし | いいえ |
| `grid_origin_x_m` | read_only | なし | いいえ |
| `grid_origin_y_m` | read_only | なし | いいえ |
| `points_topic` | read_only | なし | いいえ |
| `scan_topic` | read_only | なし | いいえ |
| `points_qos_reliability` | read_only | なし | いいえ |
| `base_frame` | read_only | なし | いいえ |
| `watchdog_hz` | read_only | なし | いいえ |
| `use_sim_time` | read_only | なし | いいえ |

`tf_timeout_s` は現状 TF の取得処理では未使用のため、変更しても動作は変わりません。
