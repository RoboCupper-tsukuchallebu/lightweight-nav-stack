# lightnav_obstacle_monitor

3D LiDAR (`PointCloud2`) と任意の 2D LiDAR (`LaserScan`、`scan_topic` が空なら無効) から、
車体周りの局所グリッド `/lightnav/local_grid` と `/lightnav/obstacle_status` を出す lifecycle ノード。
仕様は Issue #1 の契約節が正。

- 3D: 点数が `min_points_per_cell` 以上で、セル内の z の最大 − 最小、または近傍との段差が `height_diff_threshold_m` を超えたセルを占有にする。段差は、中心間のユークリッド距離が `step_neighbor_radius_m` 以下の他セルのうち点数条件を満たすセルの最小 z を G とし、対象セルの最大 z − G で判定する。近傍がなければ段差判定では占有にしない。高い側だけを占有にする。既定値では 10 度の一様な坂や 10 cm の縁石は占有にしない。
- 2D: `scan_max_use_range_m` 以内の返りだけ占有。
- stop / slow zone は膨張前のセル数で判定し、`occupied_min_frames` 連続・`clear_hold_s` 保持でヒステリシスをかける。
- 入力途絶・TF 不在は `stale_sources` と `is_input_stale` に出し、10 Hz のウォッチドッグで status を出し続ける。

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

| 環境 | 中央値 | p99 |
|---|---|---|
| Apple Silicon Mac (osx-arm64) | 0.47 ms | 1.0 ms |

目標（中央値 10 ms 未満、p99 20 ms 未満）を満たす。試験の失敗閾値は CI 向けに 100 ms。
ノード経由の受信から publish までの時間は `ObstacleStatus.processing_time_ms` に毎フレーム載る。

近距離で障害物上面だけが見えるセルも、半径内に十分な地面点があれば段差で検出する。近傍にも地面点がない場合や障害物の点自体がない場合は検出できない。
入力が途絶している間は `/lightnav/local_grid` を publish しない（controller の `grid_timeout_s` を発火させるため）。status だけを 10 Hz で出し続ける。
