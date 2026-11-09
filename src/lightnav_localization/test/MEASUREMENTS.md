# L2-a 計測条件

計測の追加は推定挙動を変更しない。既存シナリオの既定値、乱数系列、
アサーションを維持し、既知の失敗もそのまま記録する。期待した数値に
合わせてパラメーターや合否条件を変更しない。

`SyntheticWorld::add_pillars()` は明示的に呼んだ場合だけ箱を追加する。
引数は進行方向の半長、横方向の半幅、間隔、横オフセット、両側に置くか。
既定は半長・半幅 0.2 m（幅 0.4 m の四角柱）、間隔 10 m、左へ 1.5 m、片側で、
各直線区間の始点から間隔の半分（既定 5 m）おき（終点は除く）に置く。
高さは既存壁と同じ 3 m とし、四面を既存の壁区間として地図生成と
スキャン生成の両方に使う。既定の柱は `PillarsAppearInMapAndScan` が使い、
A は 1 m 幅・0.2 m 厚の箱を 1.25 m 間隔で両側に置く。

`run()` は各シナリオの最後に `[ MEASURED ]` のまとめを 1 行出し、
`sample_columns=time_sec,...` でサンプルの列名を示す。
`error_m` は最大水平位置誤差、`final_error_m` は終了時の水平位置誤差。
各サンプルは `[ SAMPLE ] scenario=<名前> ` に続けて同じ列をカンマ区切りで
1 サンプル 1 行に出し、出力のある車輪時刻ごと（0.02 s）に記録する。
pitch は絶対誤差、進行方向誤差は
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

## L2-b レビュー H1 / M1 修正（2026-11-03）

`Corrector::set_planar_target(map_to_odom, odom_to_base)` は両目標をbaseの姿勢に変換し、グラフ側のx/y/yawと直近LiDAR側のz/roll/pitchを合成してからmap→odomに戻す。呼び出し元は出力時点の最新車輪姿勢 `last_wheel_pose` を渡す。

M1は0.1秒周期の段差について入口から0.3秒後〜出口直前、出口から0.3秒後〜終了をそれぞれ全サンプル評価する。両区間の最大絶対pitch誤差が2°以内かつ件数>0を要求する。初回復帰時間は参考値として残す。既存0.2秒周期の判定は変更していない。

| シナリオ（複数実行は出力順） | L2-b 最大水平誤差 m | 修正後 m | 差 m |
|---|---:|---:|---:|
| FusionStaysCloseToTruthWhileWheelOdometryDrifts | 0.419184 | 0.419184 | +0.000000 |
| GpsBiasDoesNotMoveEstimateWhileMapTracks | 0.098199 | 0.098199 | +0.000000 |
| FallsBackToGpsOnlyWithoutMapAndRecovers | 0.487535 | 0.487535 | +0.000000 |
| OdomOnlyBecomesLostAndStopsPublishing | 0.229158 | 0.229158 | +0.000000 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (1) | 0.128468 | 0.128470 | +0.000001 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (2) | 0.170166 | 0.170170 | +0.000003 |
| RecoversFromOptimizationFailure | 0.107617 | 0.107618 | +0.000001 |
| InitializeLeavesLost | 0.229158 | 0.229158 | +0.000000 |
| GpsJumpsAreGatedInThePipeline | 0.096251 | 0.096251 | -0.000000 |
| InitialOffsetIsSlewedNotJumped | 0.900038 | 0.900038 | +0.000000 |
| ClimbsA10DegreeRamp | 0.093031 | 0.093036 | +0.000005 |
| SurvivesAPitchStep | 0.113243 | 0.113244 | +0.000001 |
| ClimbsA10DegreeRampAtTenHertz | 0.079128 | 0.079122 | -0.000006 |
| RecoversFromPitchStepWithinThreeTenthsAtTenHertz | 0.083996 | 0.083996 | +0.000000 |
| OpenCorridorIsCorrectedByGpsNotFrozenByMap | 3.124884 | 3.119231 | -0.005653 |
| MapRebuildFailureKeepsOldTarget (1) | 0.107617 | 0.107618 | +0.000001 |
| MapRebuildFailureKeepsOldTarget (2) | 0.107617 | 0.107618 | +0.000001 |
| OpenCorridorWithPreciseGps | 5.945543 | 5.945543 | +0.000000 |

| pitch 計測 | L2-b ° | 修正後 ° |
|---|---:|---:|
| 坂0.1 s・x∈[5,15]最大 | 1.437416 | 1.437847 |
| 段差入口0.3 s後以降 | 0.502322 | 0.502322 |
| 段差出口0.3 s後以降 | 0.344296 | 0.344295 |

L2-bの入口・出口の値は保存済みMEASUREDの全時系列を同じ区間で再集計した。

H1の変異確認：シグネチャだけ維持して旧map→odom合成へ戻した場合、1000 mケースでx差6.306202 m、y差1.666836 m、yaw差0.000284516 radとなり、1e-6のassertが失敗した。修正後はCorrector全6ケース通過。変異後に修正済みソースと全バイナリを復元し、ROS_DOMAIN_ID=44でCTest全量を実行した。

復元後のCTest結果：14ターゲット中13通過、localizerの既知4ケース（A、0.2秒坂、0.2秒段差、開いた廊下）のみ失敗。0.1秒坂と入口・出口の継続判定は通過。入口818サンプル、出口2265サンプルを検証した。C++レビューは必須修正なし。ROS通信試験およびclang-tidy/cppcheckは未実施。

## L2-c 事前確認で停止（2026-11-03）

`InformationLimits::degeneracy_ratio` の既定だけを0.05から0.01に変更した。
この既定は地図照合とLiDARオドメトリの両方、およびScenarioに適用される。
ROS_DOMAIN_ID=44で対象をビルドし、既存test_match_informationと開いた廊下の
パイプライン2ケースだけを実行した。ビルド成功、全量CTestは未実施。

| シーン | 固有値比（最小〜最大） | 0.01での判定 |
|---|---:|---|
| test_match_information 開廊下 | 0.022715995 | 最弱方向も非縮退 |
| localizer 開廊下・GPS σ1 m | 0.00145924276〜0.00165536393 | 最弱方向は縮退 |
| localizer 開廊下・GPS σ0.1 m | 0.00146052020〜0.00173700881 | 最弱方向は縮退 |

単体試験は400 m廊下中央で静止スキャンを同じスキャンへ自己照合する。
パイプラインは120 m廊下を走行し、0.2 m間隔で生成した地図に照合する。
照合対象と密度が異なり、同じHessianの計測ではない。

単体試験の進行方向σは0.02 mとなり、既存のσ>=1 mを満たさない。
RealHessianOfOpenCorridorWeakensSlidingDirectionOnly、および
SlidingDirectionIsWeakAtEveryEffectiveRangeの2ケースが失敗した。
後者はrange=30/40/45/50/60 mで失敗、20 mでは既存判定を通過した。
ユーザー指定の停止条件に該当するため、シーン・閾値・assertは変更していない。

| パイプライン最大水平誤差 m | L2-b修正後 | 縮退比だけ0.01 |
|---|---:|---:|
| 開廊下・GPS σ1 m | 3.119231 | 3.110401 |
| 開廊下・GPS σ0.1 m | 5.945543 | 5.891444 |

σ0.1 mの最終誤差は5.888045 m、GPS受理数は[0,10),[10,20),[20,∞)秒で
9/3/0件。これはGPSゲート等をまだ変更していない途中状態の計測である。
GPS時刻ノード廃止、trusted fixのゲート省略、矛盾率の窓、std座標変換、
縮退時の絶対観測扱い、追加単体試験とAの0.1秒版は未着手。
0.01と既存開廊下試験の要求の不整合についてユーザー判断を待つ。

## L2-c 実装・検証途中（2026-11-03）

上の「事前確認」は過去の実験記録。現在の縮退比の既定は **0.05**。GPS専用ノードを廃止し、既存の最寄りキーフレームへ車輪変位を補償して因子を追加する。時刻差が0.5 sを超える場合は棄却し、`gps_time_gap_rejection_count()`で数える。既知共分散の標準偏差が0.3 m以下ならマハラノビスゲートを省略する。矛盾は直近10件のゲート棄却率50%以上（窓が埋まってから）で検出する。位置標準偏差のx/yをmap座標に変換し、縮退照合では絶対観測時刻を更新しない。

指定ビルド成功。ROS_DOMAIN_ID=44で全14 test_*バイナリを実行し、localizer以外の13ターゲットは通過。localizerは23件中17通過・6失敗。既知4件に加え、引継ぎ差分のA・0.1秒版とGPS σ0.1 m廊下の受理率が失敗。既存の判定条件は緩めていない。ROS通信試験は未実施。

GPS入力9件、姿勢グラフ4件、0.5 s境界・車輪変位補償（map/odom間90°回転を含む）・縮退時刻の重点3件は通過。読み取りレビューでは重大指摘なし。

以下は今回の `[ MEASURED ]` 出力。L2-bはH1/M1修正後の値。

| シナリオ | L2-b最大水平誤差 m | L2-c最大水平誤差 m | 差 m |
|---|---:|---:|---:|
| FusionStaysCloseToTruthWhileWheelOdometryDrifts | 0.419184 | 0.426817 | +0.007633 |
| FusionStaysCloseToTruthAtTenHertz | 未計測 | 0.409315 | — |
| GpsBiasDoesNotMoveEstimateWhileMapTracks | 0.098199 | 0.090452 | -0.007747 |
| FallsBackToGpsOnlyWithoutMapAndRecovers | 0.487535 | 0.471446 | -0.016089 |
| OdomOnlyBecomesLostAndStopsPublishing | 0.229158 | 0.229158 | -0.000000 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (1) | 0.128470 | 0.129317 | +0.000847 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (2) | 0.170170 | 0.130539 | -0.039631 |
| RecoversFromOptimizationFailure | 0.107618 | 0.124711 | +0.017093 |
| InitializeLeavesLost | 0.229158 | 0.229158 | -0.000000 |
| GpsJumpsAreGatedInThePipeline | 0.096251 | 0.113852 | +0.017601 |
| InitialOffsetIsSlewedNotJumped | 0.900038 | 0.900038 | -0.000000 |
| ClimbsA10DegreeRamp | 0.093036 | 0.101822 | +0.008786 |
| SurvivesAPitchStep | 0.113244 | 0.127424 | +0.014180 |
| ClimbsA10DegreeRampAtTenHertz | 0.079122 | 0.087991 | +0.008869 |
| RecoversFromPitchStepWithinThreeTenthsAtTenHertz | 0.083996 | 0.091542 | +0.007546 |
| OpenCorridorIsCorrectedByGpsNotFrozenByMap | 3.119231 | 2.799519 | -0.319712 |
| MapRebuildFailureKeepsOldTarget (1) | 0.107618 | 0.124711 | +0.017093 |
| MapRebuildFailureKeepsOldTarget (2) | 0.107618 | 0.124711 | +0.017093 |
| OpenCorridorWithPreciseGps | 5.945543 | 1.663889 | -4.281654 |

| pitch計測 | L2-b ° | L2-c ° | 差 ° |
|---|---:|---:|---:|
| 坂0.1 s | 1.437847 | 1.608216 | +0.170369 |
| 段差入口 | 0.502322 | 0.520749 | +0.018427 |
| 段差出口 | 0.344295 | 0.641996 | +0.297701 |

受け入れ未達：精密GPS廊下の最終誤差は0.512125 m（<1 m）だが、受理143/200件＝71.5%で80%未満。57件が時刻差で棄却された。Eは最大誤差0.090452 mでis_gps_conflict=true。F/C/D/Iの最大水平誤差は+0.05 m以内。坂0.1 sと段差入口のpitch差は+0.2°以内だが、段差出口は+0.297701°で未達。Aは0.426817 m。追加A・0.1 s版は0.409315 mで0.3 m判定に未達（GPS時刻差棄却333/341件）。

残作業：GPSとキーフレームの位相による大量の時刻差棄却、および段差出口の劣化を調査する。未生成の将来キーフレームを待つ処理は現在なく、GPS処理時点で存在する最寄りノードを選ぶ。閾値変更・数値合わせは行っていない。commit/pushなし。

## L2-c キーフレーム時間比較の修正（2026-11-03）

修正前の追加計測で、精密GPS廊下は最大キーフレーム間隔1.2 s、時刻差棄却57件、ゲート棄却0件。0.1 s周期の段差は最大間隔1.1 s、時刻差棄却63件、ゲート棄却0件だった。浮動小数の減算で1.0 s境界を取りこぼすことを再現したため、経過時間の比較に1 µsの許容を追加した。GPS時刻差上限0.5 sは変更していない。

本実装は処理時点で存在する最寄りキーフレームを選ぶ（未来のキーフレームを待たない）。したがって「前後の最寄りなら最大0.6 s」という説明と厳密には異なる。今回の試験はGPSが毎秒+0.26 s、最初のスキャンが+0.2 sまたは+0.1 sで、キーフレーム時刻のずれがGPSとの位相を変えた。

追加の `gps_gate_rejection_count()` と `max_keyframe_interval_sec()` をMEASUREDに出力する。Aの0.1 s版は計測専用とし、縮退比0.01前提の0.3 m判定はL2-dでユーザー判断後に入れる旨をコメントした。既存試験の判定は変更していない。

| シナリオ | L2-b最大誤差 m | 修正前L2-c m | 今回 m | L2-b差 m |
|---|---:|---:|---:|---:|
| FusionStaysCloseToTruthWhileWheelOdometryDrifts | 0.419184 | 0.426817 | 0.386569 | -0.032615 |
| FusionStaysCloseToTruthAtTenHertz | 未計測 | 0.409314822 | 0.380179 | — |
| GpsBiasDoesNotMoveEstimateWhileMapTracks | 0.098199 | 0.090452 | 0.102417 | +0.004218 |
| FallsBackToGpsOnlyWithoutMapAndRecovers | 0.487535 | 0.471446 | 0.474719 | -0.012816 |
| OdomOnlyBecomesLostAndStopsPublishing | 0.229158 | 0.229158 | 0.222933 | -0.006225 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (1) | 0.128470 | 0.129317 | 0.127235 | -0.001235 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose (2) | 0.170170 | 0.130539 | 0.144845 | -0.025325 |
| RecoversFromOptimizationFailure | 0.107618 | 0.124711 | 0.122387 | +0.014769 |
| InitializeLeavesLost | 0.229158 | 0.229158 | 0.222933 | -0.006225 |
| GpsJumpsAreGatedInThePipeline | 0.096251 | 0.113852 | 0.125302 | +0.029051 |
| InitialOffsetIsSlewedNotJumped | 0.900038 | 0.900038 | 0.900038 | -0.000000 |
| ClimbsA10DegreeRamp | 0.093036 | 0.101822 | 0.100641 | +0.007605 |
| SurvivesAPitchStep | 0.113244 | 0.127424 | 0.112454 | -0.000790 |
| ClimbsA10DegreeRampAtTenHertz | 0.079122 | 0.087991 | 0.100052 | +0.020930 |
| RecoversFromPitchStepWithinThreeTenthsAtTenHertz | 0.083996 | 0.091542 | 0.090762 | +0.006766 |
| OpenCorridorIsCorrectedByGpsNotFrozenByMap | 3.119231 | 2.799519 | 2.971750 | -0.147481 |
| MapRebuildFailureKeepsOldTarget (1) | 0.107618 | 0.124711 | 0.133176 | +0.025558 |
| MapRebuildFailureKeepsOldTarget (2) | 0.107618 | 0.124711 | 0.133176 | +0.025558 |
| OpenCorridorWithPreciseGps | 5.945543 | 1.663889 | 0.362240 | -5.583303 |

| 計測 | L2-b | 修正前L2-c | 今回 |
|---|---:|---:|---:|
| 坂0.1 s pitch ° | 1.437847 | 1.60821593 | 2.40718397 |
| 段差入口 pitch ° | 0.502322 | 0.520749099 | 0.819256209 |
| 段差出口 pitch ° | 0.344295 | 0.641996445 | 0.465385565 |
| 精密GPS廊下 最終誤差 m | 未記録 | 0.512124583 | 0.242823182 |
| 精密GPS廊下 受理率 | 未記録 | 0.715 | 1 |

| シナリオ | GPS投入 | 時刻差棄却 | ゲート棄却 | 最大キーフレーム間隔 s |
|---|---:|---:|---:|---:|
| FusionStaysCloseToTruthWhileWheelOdometryDrifts | 341 | 0 | 0 | 1 |
| FusionStaysCloseToTruthAtTenHertz | 341 | 0 | 0 | 1 |
| GpsBiasDoesNotMoveEstimateWhileMapTracks | 71 | 0 | 71 | 1 |
| FallsBackToGpsOnlyWithoutMapAndRecovers | 341 | 0 | 0 | 1 |
| OdomOnlyBecomesLostAndStopsPublishing | 0 | 0 | 0 | 1 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose | 71 | 0 | 0 | 1 |
| FalseConvergenceAlongCorridorDoesNotJumpThePose | 71 | 0 | 0 | 1 |
| RecoversFromOptimizationFailure | 71 | 0 | 0 | 1 |
| InitializeLeavesLost | 0 | 0 | 0 | 1 |
| GpsJumpsAreGatedInThePipeline | 71 | 0 | 14 | 1 |
| InitialOffsetIsSlewedNotJumped | 71 | 0 | 0 | 1 |
| ClimbsA10DegreeRamp | 71 | 0 | 0 | 1 |
| SurvivesAPitchStep | 71 | 0 | 0 | 1 |
| ClimbsA10DegreeRampAtTenHertz | 71 | 0 | 0 | 1 |
| RecoversFromPitchStepWithinThreeTenthsAtTenHertz | 71 | 0 | 0 | 1 |
| OpenCorridorIsCorrectedByGpsNotFrozenByMap | 200 | 0 | 0 | 1 |
| MapRebuildFailureKeepsOldTarget | 71 | 0 | 0 | 1 |
| MapRebuildFailureKeepsOldTarget | 71 | 0 | 0 | 1 |
| OpenCorridorWithPreciseGps | 200 | 0 | 0 | 1 |

GPSノード廃止のpitchへの影響経路：車輪・LiDAR因子の始点が直前GPSノードから前キーフレームへ変わり、固定tilt標準偏差を持つ車輪因子の構成も変わる。GPS更新自体はpitchを保持するが、次キーフレームの全姿勢更新とLiDARサブマップ更新から影響し得る。これはコード上の経路の確認であり、単独の因果を確定する比較実験は行っていない。

指定ビルド成功。ROS_DOMAIN_ID=44でtest_localizer全量を実行。丸め境界の追加試験も通過。読み取りレビューに必須修正なし。commitなし。

今回の判定：精密GPS廊下は受理200/200＝100%、最終誤差0.242823 mで達成。段差出口は0.465386°、L2-b差+0.121091°で許容内に改善した。ただし入口は0.819256°（+0.316934°）、坂0.1 sは2.407184°（+0.969337°）で未達。時間判定だけの変更で出口が改善するため、前回の出口悪化をGPSノード廃止だけに帰属できない。
全24件中18通過、6失敗。既知4件に加えInitialOffsetIsSlewedNotJumpedとClimbsA10DegreeRampAtTenHertzが失敗した。これらの新たな失敗は未修正で、判定を緩めず残す。今回の3項目を超える修正は行っていない。

## L2-c 新規失敗の切り分けとGPS水平因子修正（2026-11-03）

各変異は開始時点のコードから一つだけ変更し、指定ビルド後にROS_DOMAIN_ID=44で初期オフセット・坂0.1 s・段差0.1 sの3件を実行した。各回で元のコードへ戻した。GPSノード復元では旧0.2 s境界と追加ノード経路を復元し、その他のゲート・矛盾検知・1 µs許容は維持した。最後に変異を全て除去してGPS因子だけxy専用に修正した。

| 条件 | I 最終誤差 m | 坂0.1 s pitch ° | 段差入口 ° | 段差出口 ° |
|---|---:|---:|---:|---:|
| 変更前（今回開始時点） | 0.365145 | 2.407184 | 0.819256 | 0.465386 |
| 時間判定だけ旧比較へ戻す | 0.041586 | 1.608216 | 0.520749 | 0.641996 |
| trusted fixにもゲートを戻す | 0.365145 | 2.407184 | 0.819256 | 0.465386 |
| 縮退照合でもmark_absoluteへ戻す | 0.365145 | 2.407184 | 0.819256 | 0.465386 |
| GPS時刻ノードだけ復元 | 0.397769 | 2.297942 | 0.723483 | 0.573659 |
| 最終修正：GPS因子をxy専用化 | 0.047251 | 1.780980 | 0.846832 | 0.579535 |

初期オフセットの失敗は終了時位置誤差であり、速度上限ではなかった。全変異・修正後とも最大出力速度は0.830 m/sで既存判定を通過した。この3シナリオはtrusted fixを使わないため、trustedゲート復元の結果が同じなのは妥当。slew経路は変更していない。

時間判定修正がGPS受理数と因子配置を変え、既存の3D GPS因子の影響を顕在化させた。GPSFactorはzも拘束し、坂の合成GPSは高度0のままなので、LiDARで得た高さと矛盾する。さらにHuberの重みが3D残差全体で計算され、z残差がxyの重みにも影響していた。GPS専用ノード復元だけでは新規失敗は解消しなかった。

`HorizontalGpsFactor`を内部実装し、mapのx/y残差だけを使用する。JacobianはPose3::translationの先頭2行で、傾いたbody座標からmapへの回転を維持する。xyの標準偏差とHuberは従来値を使用し、GPS高度・sigma_zは因子に使用しない。GPSからz/roll/pitchを直接拘束する観測を除去したが、他因子との相関による間接影響までゼロにするものではない。

| 受け入れ | L2-b | 修正後 | 差 | 結果 |
|---|---:|---:|---:|---|
| 坂0.1 s pitch ° | 1.437847 | 1.780980 | +0.343133 | 既存2°試験は通過、+0.2°条件は未達 |
| 段差入口 pitch ° | 0.502322 | 0.846832 | +0.344510 | 既存2°試験は通過、+0.2°条件は未達 |
| 段差出口 pitch ° | 0.344295 | 0.579535 | +0.235240 | 既存2°試験は通過、+0.2°条件は未達 |

指定ビルド成功。pose_graph全5件通過（高度だけ異なるGPSを傾いた姿勢に与えても高さ・傾きが変わらない回帰試験を追加）。対象localizer3件は全通過。読み取りレビューに必須修正なし。残るpitch差の原因は未確定で、閾値調整や既存試験の弱体化はしていない。commitなし。

最終コードのtest_localizer全量は24件中20通過、失敗は既知4件のみ。InitialOffsetIsSlewedNotJumpedとClimbsA10DegreeRampAtTenHertzの新規失敗は解消した。段差の入口・出口を含むL2-b比+0.2°条件は上表のとおり未達。

## L2-c レビュー決定反映（2026-11-04）

GPS時刻差上限を1.2 sへ変更。trusted fixも絶対距離2 m超は棄却し、通常マハラノビスまたは絶対ゲートの棄却相当判定を矛盾窓へ記録する。NoFix/HDOP/Invalid/時刻差棄却は窓に含めない。窓の意味は地図照合状態によらない全区間のGPS矛盾。縮退比0.05は維持。

localizerはStateInputs::allow_uncertain_map_trackingを有効にし、新鮮な地図照合中だけposition_stdによるLOSTを抑止する。StateMachine単体の既定動作と既存試験を維持し、地図照合が古くなると通常のLOST判定へ戻る。公開するposition_stdは抑制しない。

| 指標 | L2-c（前回GPS xy化後） | 今回 |
|---|---:|---:|
| A 最大誤差 m | 0.385914306 | 0.387688344 |
| 坂0.1 s pitch ° | 1.78097964 | 1.68009851 |
| 精密GPS廊下 最大誤差 m | 0.361322971 | 0.361322212 |
| 精密GPS廊下 最終誤差 m | 0.243820959 | 0.242239545 |
| 精密GPS廊下 受理率 | 1 | 1 |
| E 最大誤差 m | 0.102417467 | 0.102417467 |
| E 矛盾検知 | 1 | 1 |
| I 最終誤差 m | 0.0472510491 | 0.0465487853 |

1 Hz GPSの位相0.6/0.7/0.8/0.9 sは停止状態で各10/10件受理（100%）。1.2 s境界受理・1.21 s棄却の試験通過。σ0.1 m・3 m偏り4件とNoFix6件を混在させ、窓4件で矛盾検知する試験が通過。trusted受理でも通常ゲート棄却相当が返る単体試験も通過。

10 Hz計測：停止中、最初のキーフレームだけを生成し、その後のスキャンを与えずt=0.0〜1.1 sで12 fixを投入した結果、1キーフレームに12因子が追加された。通常の1 Hzスキャン運用を測った値ではなく、上限1.2 s内で同一ノードへ積み重なるケースの計測である。相関・過小σへの対策は今回未実施。通常シナリオの1 Hz GPSは最大1因子/キーフレームだった。

ビルド成功。全14 test_*をROS_DOMAIN_ID=44で実行し、localizerとpose_graph以外の12ターゲットは通過。pose_graphのGpsJumpsOf20MetersDoNotMoveTheEstimateが新規失敗：外れ値あり最終誤差1.050651 m（上限0.8）、cleanとの差0.465052 m（上限0.3）。絶対2mゲートによる観測棄却が追加された状態での失敗であり、判定は緩めていない。

150 m廊下の最初の試験（車輪倍率1.05）は最終出力231.2 sで停止し、地図照合が新鮮であり続ける条件を満たせなかった。車輪倍率1.0で不確かさ増加の経路を分離した追加条件を再検証する。既存シナリオの条件は変更していない。

## L2-c trusted専用の復帰可能な距離ゲート（2026-11-04）

受理判定の絶対距離ゲートをtrusted fixだけに限定し、閾値を`max(gps_absolute_gate_m, gate_mahalanobis × predicted_position_std_m)`とした。通常GPSは従来のマハラノビスゲートだけで判断する。矛盾窓へ記録する`gate_would_reject`は固定2 m超または通常マハラノビス棄却相当のまま維持する。

追加試験：推定位置が3 mずれ、推定σ=1.5 mのとき精密GPS σ=0.1 mを受理する。同じ3 m差でも推定σ=0.1 mでは棄却し、5 m差・推定σ=1.5 mでも棄却する。通常GPS σ=1 mは2.5 m差をマハラノビス条件内で受理する。既存の距離棄却試験は新仕様と逆になる推定σ=10 mを0.1 mに変更し、棄却アサーションを保持した。NoFix混在で精密GPSの3 m偏りを棄却・矛盾検知するパイプライン試験も通過した。

| 指標 | L2-c（GPS xy化後） | 全fixへ2 mゲート | 今回 |
|---|---:|---:|---:|
| A 最大誤差 m | 0.385914306 | 0.387688344 | 0.385914306 |
| 坂0.1 s 最大誤差 m | 0.0954729023 | 0.0875354302 | 0.0954729023 |
| 坂0.1 s pitch ° | 1.78097964 | 1.68009851 | 1.78097964 |
| 段差入口 pitch ° | 0.846831606 | 0.634653568 | 0.846831606 |
| 段差出口 pitch ° | 0.579534517 | 0.472888065 | 0.579534517 |
| 通常GPS廊下 最大誤差 m | 2.97177145 | 5.05381868 | 2.97177145 |
| 精密GPS廊下 最大誤差 m | 0.361322971 | 0.361322212 | 0.361322971 |
| 精密GPS廊下 最終誤差 m | 0.243820959 | 0.242239545 | 0.243820959 |
| 精密GPS廊下 受理率 | 1 | 1 | 1 |
| E 最大誤差 m | 0.102417467 | 0.102417467 | 0.102417467 |
| E 矛盾検知 | 1 | 1 | 1 |
| I 最終誤差 m | 0.0472510491 | 0.0465487853 | 0.0472510491 |

150 mの縮退廊下：車輪倍率1.05では地図照合が途切れ、最終出力231.2 sで試験が通らなかった。新鮮な地図照合が途切れたらLOST抑止を解除し、従来どおりodom_onlyの距離上限でLOSTになるのは仕様どおり（位置標準偏差など他の既存条件も有効）。今回は車輪倍率1.0の試験を維持し、全量実行でも通過した。これは新鮮な地図照合中のLOST抑止の確認であり、1.05倍の車輪ドリフトでも150 m追跡できることを保証する試験ではない。

指定ビルド成功。ROS_DOMAIN_ID=44でtest_localizer全28件中24通過、失敗は既知4件のみ。test_pose_graphは5/5、test_gps_inputは12/12、test_state_machineは5/5通過。GpsJumpsOf20MetersDoNotMoveTheEstimateの新規失敗は解消した。既存の閾値は緩めず、試験の削除・skipなし。README更新、読み取りレビューで必須修正なし。commitなし。

## L2-d 試験シーンと判定の見直し（Issue #25, 2026-11-04）

推定の `.cpp/.hpp` は変更していない。試験のシーンと判定だけを Issue #25 の
承認済みの値に合わせた。スキャン周期は A・坂・段差を実機どおり 0.1 s にした。
古い 5 Hz の重複試験（A、坂、段差）は削除し、0.1 s の試験を判定付きで残した。
`test_localizer` は 28 件から 25 件になった。`test_pose_graph` は触っていない。

### A のシーン

A のコース（70 m + 60 m + 70 m、90 度旋回 2 回）に、進行方向の面を持つ箱を
置いた。箱は 1 m 幅（横方向）・0.2 m 厚（進行方向）、両側、間隔 1.25 m、
各直線区間の始点から間隔の半分（0.625 m）おき。まず Issue の例どおり
1 m 幅・0.2 m 厚を 10 m 間隔で片側に置くと、`[ SAMPLE ]` の
`eigen_ratio` は最小 0.0025 で 0.02 に届かなかった。両側・2 m 幅・10 m 間隔
でも最小 0.008 だった。間隔を 1.25 m まで詰めると全サンプルで 0.02 以上に
なり、最小 0.0341 を実測した。寸法はこの計測で決めた。

| 計測 | 実測 |
|---|---:|
| A 最大水平誤差（基準 < 0.2 m） | 0.0677990773 m |
| A 最大 yaw 誤差 | 0.94° |
| A 車輪のみ終了誤差 | 13.50 m |
| A `eigen_ratio` 最小 | 0.0341 |
| A scan 最大処理時間 | 374.7 ms（既存基準 < 450 ms） |

処理時間はマシン負荷で変動する（別実行では 396.9 ms）。

### 坂・段差

| 計測 | 実測 | 基準 |
|---|---:|---:|
| 坂 pitch 最大誤差（真値 x ∈ [5, 15] m） | 1.76054764° | ≤ 2° |
| 坂 最大水平誤差 | 0.0896834433 m | < 0.2 m |
| 段差 2° 以内への整定時間 | 0.166666667 s | ≤ 0.3 s |
| 段差 最大水平誤差 | 0.089684841 m | < 0.2 m |

段差は「入口から 2° 以内に戻るまでの時間 ≤ 0.3 s」に加え、入口・出口それぞれ
0.3 s 後以降の全サンプルが 2° 以内に留まることで判定する。実測は入口
0.650685547°（818 サンプル）、出口 0.435576943°（2265 サンプル）で、
初回復帰は 0.166666667 s。

### 開いた廊下

どちらも 120 m 直線、端にキャップなし、0.5°/10 m の yaw ドリフト。GPS σ 1 m の
版は Issue の車輪モデル（スケール 1 %）、GPS σ 0.1 m の版は設計どおり 5 %。

| 計測 | 実測 | 基準 |
|---|---:|---:|
| 車輪 1 %・GPS σ 1 m 最大水平誤差 | 0.418847095 m | < 1.0 m |
| 同 終了時誤差 | 0.353318743 m | < 1.0 m |
| 同 車輪のみ終了誤差 | 6.41 m | — |
| 車輪 5 %・GPS σ 0.1 m 最大水平誤差 | 0.361247304 m | < 0.7 m |
| 同 終了時誤差 | 0.242364877 m | < 0.7 m |
| 同 GPS 受理率 | 1.00 | ≥ 0.8 |

GPS σ 0.1 m の版は設計（#2 の見直し案 1-4 / L2-c）どおり車輪 5 % のまま。
正確な GPS が廊下でゲートに弾かれ、車輪のずれどおりに発散する欠陥を確かめる
試験なので、車輪 1 % には弱めない。受け入れ試験としてスイートを `Localizer`
に移し、`OpenCorridorWithPreciseGpsAndFivePercentWheel` とした。

### 判定の確認

`pixi run build` 成功。`pixi run lint`（ruff check src）も成功。
`test_localizer` 全 25 件を 1 回実行し、25 件すべて通過した。しきい値は
Issue #2 の承認値から緩めていない。

## 大域初期化の枝刈りに残るリスク（Issue #70 R-1）

`too_many_peaks` は非極大値抑制後の粗い 2D 点数を比較する。候補を
切り捨てる場合に言えるのは、その点数が 1 位より candidate margin を
超えて低いことだけである。3D の確かめでは順位が逆転し、切り捨てた
峰がよく合う可能性は残る。実データで誤受理がないことの確認は PR-3 の
受け入れ条件 R3 で行う。

## 大域初期化の暫定試験（ビルド機）

中間版（`91b6875` 相当のソース）で `docker build -f docker/Dockerfile --target test` と
`docker run --rm --shm-size=1g` を実行。大域初期化 gtest は 9 件成功、合計 2 ms。
全体試験の既存性能試験では PreparedTarget 構築が 204.7 ms で、上限 180 ms を超過した。
この値は G10 の測定ではなく、最新 head の試験結果でもない。2,500 m² の探索時間、
段階別時間・メモリ、X13 との比較は未計測。

続いて `cd8ecf8` のソースを同じ手順で再実行した。Docker ビルドは 4 分 17 秒、
コンテナ内の全体試験は 10 分 29 秒、398 件すべて成功。大域初期化 gtest は
10 件成功、合計 1 ms。G10 の計測は引き続き必要。

### 2026-11-09 差分試験と単独再計測

対象 head: `2f7f0e1`。ビルド機の `dev.sh test gi1 lightnav_localization` は
9 分 34 秒、集計 54 件・失敗 4 件（CTest と個別結果を含む集計）だった。
大域初期化の 20 件は全て成功し、G6 は 1.141 秒だった。失敗は性能試験、
Localizer の融合試験、localization 起動終了試験（集計には重複を含む）。
性能試験の PreparedTarget 構築は 215.0 ms（52,020 点）で、180 ms の上限を超えた。

単独再計測時の負荷平均は 9.99 / 8.49 / 11.17、PreparedTarget は 218.2 ms。
負荷が高い状態で上限を超えたため、無負荷時の上限達成とは判定できず、
負荷が低い状態で再計測が必要。受理試験の単独実行は負荷平均
9.99 / 8.52 / 11.17、5 姿勢を合わせて 9.877 秒で成功した。各姿勢の
所要時間と G10 の内訳・X13 との比較は未計測。

対象 head `7147c9b` で再実行した `dev.sh test gi1 lightnav_localization` は
10 分 18 秒、集計 54 件・失敗 0 件だった（集計には CTest と個別結果を含む）。
G7 の試作では、正しい場所を含まない窓の中に異なる通路を置くと `ambiguous` で
拒否された。仕様が要求する `low_score` / `outside_gnss_window` ではなく、
この試作は採用していない。GNSS 窓とレバーアームの実装を含めて確認が必要。

### 2026-11-09 GNSS 窓と G10 の追加計測

試験対象ソース `1cdafc7`（後の `3ac22e7` は G11・G12 試験のみ追加）。
`dev.sh test gi1 lightnav_localization` は 13 分 38 秒、CTest 24 件中 2 件失敗。
性能試験の PreparedTarget は 241.4 ms（上限 180 ms）、Localizer の最大 scan
処理は 542.6 ms（上限 450 ms）。両方とも時間の判定で失敗した。大域初期化は
23 件すべて成功し、G13 の拒否理由は `ambiguous`。開始時の負荷平均は
17.27 / 11.79 / 9.93。集計上は CTest と個別結果が重複する。

PreparedTarget の単独再測定は負荷平均 8.48 / 11.89 / 12.71 のとき 169.9 ms。
低負荷（1 分平均 4 未満）での実測ではないため、負荷依存の値として扱う。

G10 は 50 m × 50 m の探索範囲、地図はその一部にある合成通路であり、
地図全域に構造物がある 2,500 m² のケースを代表するものではない。上記の
パッケージ試験では全体 7.14543 s、場の構築 0.019839 s、粗い探索 6.03951 s、
細かい探索 0.213262 s、確かめ 0.871904 s。場は 549,081 セルで、距離配列
自体は double で約 4.19 MiB（作業領域やプロセスの最大メモリは未計測）。
候補別 PreparedTarget は 0.0349308〜0.0357236 s、二段照合は
0.0226844〜0.121284 s。X13 との比較は未実施。

`3ac22e7` の追加 G11・G12 は個別に 2 件とも成功、2.763 s。G11 の囮は
通常の高さの部屋であり、仕様の「帯のみに一致する囮で相手の順位を下げる」
条件の実証は未完了。G12 は地図外の通路で拒否するが、scan-to-map 単独なら
受理する条件を実証していない。全パッケージ試験にはこの 2 件を含まない。
