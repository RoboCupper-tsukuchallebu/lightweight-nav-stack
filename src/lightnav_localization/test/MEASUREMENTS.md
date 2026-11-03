# L2-a 計測条件

計測の追加は推定挙動を変更しない。既存シナリオの既定値、乱数系列、
アサーションを維持し、既知の失敗もそのまま記録する。期待した数値に
合わせてパラメーターや合否条件を変更しない。

`SyntheticWorld::add_pillars()` は明示的に呼んだ場合だけ柱を追加する。
各直線区間の始点から 10 m おき（終点は除く）、進行方向の左側へ 1.5 m の
位置に幅 0.4 m の四角柱を置く。高さは既存壁と同じ 3 m とし、四面を
既存の壁区間として地図生成とスキャン生成の両方に使う。
既存試験では柱を追加しない。

`run()` は各シナリオの最後に `[ MEASURED ]` の 1 行を出す。
`error_m` は最大水平位置誤差、`final_error_m` は終了時の水平位置誤差。
`samples=` はヘッダーに続くセミコロン区切りのサンプル列で、出力のある
車輪時刻ごと（0.02 s）に記録する。pitch は絶対誤差、進行方向誤差は
真値の車体 x 軸への射影（符号付き）。固有値比と最弱軸は最後の照合値を保持し、
未計測は nan。軸は scan 座標系で符号不定。GPS 受理数は受理した測定の
時刻で [0,10), [10,20), [20,∞) s に分ける。

GPS 雑音は従来の `gps_noise_m`、明示的な受信機の標準偏差は
`gps_sigma_m`。既定の 1 m では従来どおり `gps_noise_m` に連動する。
追加の `OpenCorridorWithPreciseGps` だけ両者を 0.1 m に設定する。
既存の開いた廊下試験は 1 m のまま。

## 2026-11-03 の実測

ROS_DOMAIN_ID=43、既存 Release ビルド、CTest 全 14 ターゲットを 1 回実行。
13 ターゲット通過、localizer の既知 4 ケースのみ失敗。
追加した GPS 0.1 m と柱の検証は通過。

| 計測 | 実測 |
|---|---:|
| A 最大水平誤差 | 1.1193288 m |
| 坂 pitch 最大誤差 | 9.81107957° |
| 段差 pitch 最大誤差 | 10.0195667° |
| 開いた廊下 最大水平誤差（GPS 1 m） | 1.93583389 m |
| A の x=33〜43 m 固有値比 | 0.00157191503〜0.00171059137 |
| A の最初の曲がり角の 25 m 先（t=162.1〜162.4 s） | 0.0272320299 |
| test_match_information 開いた廊下 固有値比 | 0.022715995 |
| GPS 0.1 m 開いた廊下 20 s 以降の受理数 | 0 |

開いた廊下の Hessian 単体試験は期待された 0.0015 と一致しなかった。
実際の出力を記載し、シーンや推定パラメーターの調整はしていない。
最弱軸は (-0.999999998, -0.00000870280663, 0.0000643808744)。
`test_match_information` は LiDAR odometry の自己照合シーン、
`test_localizer` は地図照合なので、同一の Hessian を計測しているわけではない。

## L2-b 実測（2026-11-03）

予測起点は `Corrector::target() * odom_now`。古いスキャン姿勢は削除した。前回スキャンの車輪姿勢は `last_scan_odom_for_gate` としてゲート距離計算だけに残し、距離の意味を維持する。GPS 最適化後は `set_planar_target()` で x/y/yaw のみ更新し、キーフレームでは全姿勢を更新する。縮退比・GPS 時刻ノード・ゲート条件は変更していない。

ROS_DOMAIN_ID=44。CTest 全14ターゲットを1回実行し13通過。localizer は19ケース中15通過、従来の4ケースが失敗。既存の閾値・試験は変更していない。Corrector の平面合成と0.1秒周期の坂・段差の追加試験は通過。

| シナリオ（複数実行は出力順） | L2-a 最大水平誤差 m | L2-b 最大水平誤差 m | 差 m | +0.05 m以内 |
|---|---:|---:|---:|:---:|
| FusionStaysCloseToTruthWhileWheelOdometryDrifts | 1.119329 | 0.419184 | -0.700145 | ○ |
| GpsBiasDoesNotMoveEstimateWhileMapTracks | 0.095388 | 0.098199 | +0.002811 | ○ |
| FallsBackToGpsOnlyWithoutMapAndRecovers | 1.118415 | 0.487535 | -0.630880 | ○ |
| OdomOnlyBecomesLostAndStopsPublishing | 0.228938 | 0.229158 | +0.000220 | ○ |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (1) | 0.126764 | 0.128468 | +0.001705 | ○ |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (2) | 0.157280 | 0.170166 | +0.012886 | ○ |
| RecoversFromOptimizationFailure | 0.107166 | 0.107617 | +0.000452 | ○ |
| InitializeLeavesLost | 0.228938 | 0.229158 | +0.000220 | ○ |
| GpsJumpsAreGatedInThePipeline | 0.106678 | 0.096251 | -0.010427 | ○ |
| InitialOffsetIsSlewedNotJumped | 0.900038 | 0.900038 | +0.000000 | ○ |
| ClimbsA10DegreeRamp | 0.096070 | 0.093031 | -0.003038 | ○ |
| SurvivesAPitchStep | 0.113686 | 0.113243 | -0.000443 | ○ |
| OpenCorridorIsCorrectedByGpsNotFrozenByMap | 1.935834 | 3.124884 | +1.189050 | × |
| MapRebuildFailureKeepsOldTarget (1) | 0.107166 | 0.107617 | +0.000452 | ○ |
| MapRebuildFailureKeepsOldTarget (2) | 0.107166 | 0.107617 | +0.000452 | ○ |
| OpenCorridorWithPreciseGps | 4.395859 | 5.945543 | +1.549684 | × |

A は0.419183677 mでL2-bの0.45 m条件を満たすが、既存の0.3 m判定では失敗する。0.1秒周期の坂は1.43741604°、段差の2°以内への初回復帰は0.166666667 s。0.2秒周期の坂は2.44695029°、段差の最大誤差は10.0192687°で既存判定を満たさない。

復帰時間は真値が x=5 に達する `5.0/kSpeed` から計測し、最初の観測ですでに2°以内なら0。坂区間内で戻らなければinf、非段差はnan。復帰後の再逸脱を判定する値ではない。

変異確認は一時的に個別の変更を外して実行した。再同期を外した A は1.11932881 m（1.0 m超）、GPS後の目標合成を外した0.1秒周期の坂は9.44349552°（9°超）。確認後、変更後のソースとtest_localizerバイナリを復元した。

開いた廊下の2ケースは+0.05 m条件を満たさない。数値合わせやL2-cの条件変更は行っていない。
