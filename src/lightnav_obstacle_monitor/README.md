# lightnav_obstacle_monitor

3D LiDAR (`PointCloud2`) と任意の 2D LiDAR (`LaserScan`、`scan_topic` が空なら無効) から、
車体周りの局所グリッド `/lightnav/local_grid` と `/lightnav/obstacle_status` を出す lifecycle ノード。
仕様は Issue #1 の契約節が正。

- 3D: セルごとの z の最大 - 最小が `height_diff_threshold_m` を超え、点数が `min_points_per_cell` 以上なら占有（10 度の傾斜や 10 cm の縁石では立たない）。
- 2D: `scan_max_use_range_m` 以内の返りだけ占有。
- stop / slow zone は膨張前のセル数で判定し、`occupied_min_frames` 連続・`clear_hold_s` 保持でヒステリシスをかける。
- 入力途絶・TF 不在は `stale_sources` と `is_input_stale` に出し、10 Hz のウォッチドッグで status を出し続ける。

## 処理時間の実測

合成点群（32 リング x 2160 方位 = 69,120 点、HDL-32E 相当、地面 + 円柱障害物）1 フレームについて、
TF 適用・投影・占有判定・膨張・zone 集計・`OccupancyGrid` 構築までの 200 回計測
（ROS の通信を除く純ロジック部分、`test_frame_processor`、Release ビルド）。

| 環境 | 中央値 | p99 |
|---|---|---|
| Apple Silicon Mac (osx-arm64) | 0.47 ms | 1.0 ms |

目標（中央値 10 ms 未満、p99 20 ms 未満）を満たす。試験の失敗閾値は CI 向けに 100 ms。
ノード経由の受信から publish までの時間は `ObstacleStatus.processing_time_ms` に毎フレーム載る。
