# bag_replay

保存した rosbag を `localization_node` に流して mapping（または localization）する道具です（Issue #43）。ビルド機で使います。
実機（ドメイン 42）・GUI（43）・試験（44）には繋がらず、ドメインは **61** で走ります。

- 再生は `ros2 bag play --clock --rate R --start-offset S`、ノードは `use_sim_time:=true` で動きます（`docker/config/*.yaml` は編集しません。上書きは `-p` で渡します）。
- **実時間より速い再生（`--rate 2`、`4`）は、ビルド機ではノードの処理が間に合う範囲でだけ使えます。** 室内の bag（`drive-20261109-024733`、
  オフセット 375 秒・360 秒分、mapping、#45 の修正を取り込んだ develop）を 1・2・4 倍で 1 本ずつ順に取り直した結果:

  | 倍率 | キーフレーム | map 点 | 経路長 | 1 倍とのキーフレーム位置差（平均 / 最大） | 動いている区間の `INPUT_STALE` | scan_ms p95 |
  |---|---|---|---|---|---|---|
  | 1 | 191 | 17865 | 35.15 m | - | 3 / 771（0.4%） | 19.8 ms |
  | 2 | 191 | 17865 | 35.15 m | 0.00 m / 0.00 m（1 倍と同じ） | 3 / 679（0.4%） | 19.9 ms |
  | 4 | 185 | 19860 | 35.11 m | 0.63 m / 3.24 m | 1162 / 1229（95%） | 25.4 ms |

  **2 倍までは 1 倍と同じ地図になり、4 倍は崩れます**（4 倍はやり直すたびに結果が変わりました: キーフレーム 167〜185）。
  4 倍で `INPUT_STALE` が増えるのは、ノードの処理が入力に追いつかないためと考えられます（4 倍の別の実験で `localization_node` の主スレッドが
  CPU 100% に張り付くのは見ましたが、取りこぼしたスキャンの数そのものは測っていません。`summary.json` の `aligned_scans_published` は 1 倍で 1870、
  2 倍で 1390、4 倍で 1323 でした）。**まず 1 倍か 2 倍で使い、4 倍以上は 1 倍の結果と `summary.json` の `status_states` を比べてから使ってください。**
  実験は他の作業と同居したビルド機で行いました（理想条件ではありません）。
- 再生するのは `/velodyne_points` `/wheel/odom` `/tf` `/tf_static` だけです。`--start-offset` で飛ばした `/tf_static` は、bag のすべての `/tf_static` メッセージを 1 つにまとめて出し直します。
- 終わると SIGINT でノードを止め、地図（`map.pcd` `map.yaml` `trajectory.csv`）を保存し、上面図の PNG と数値を出します。
- `foxglove_bridge`・`matplotlib` は **`replay` ターゲットだけ**に入ります（ロボットの runtime イメージには入りません。Foxglove は利用者の明示の依頼）。

## 使い方（ビルド機）

```bash
tools/bag_replay/run_replay.sh --build --bag ~/bags/<bag のディレクトリ> --out ~/work/run-4x \
  --mode mapping --rate 4 --start-offset 375
```

| 引数 | 既定 | 意味 |
|---|---|---|
| `--bag` / `--out` | 必須 | bag（読み取り専用）と出力先 |
| `--build` | なし | `docker build --target replay` をやり直す（イメージが無ければ自動） |
| `--mode` | `mapping` | `mapping` / `localization`（後者は `--map-file` `--map-yaml` をコンテナ内のパスで渡す） |
| `--rate` / `--start-offset` / `--duration` | 1 / 0 / 0 | 再生倍率、bag の先頭から飛ばす秒、再生する bag 上の秒（0 = 最後まで） |
| `--params-file` | `/ws/config/localization_indoor.yaml` | 室内（GNSS なし）。屋外は `/ws/config/robot.yaml` |
| `--node-param NAME:=VALUE` | | ノードの追加パラメータ（繰り返し可） |
| `--foxglove` / `--foxglove-port` | なし / 8765 | `foxglove_bridge` を起動しポートを公開する |
| `--foxglove-bind` | 0.0.0.0 | `run_replay.sh` のみ。ポートを公開するホスト側アドレス（`127.0.0.1` にして SSH トンネルで繋ぐこともできる） |
| `--hold-seconds` | 0 | 再生後、保存の前にこの秒数だけノードと bridge を生かす（Foxglove で見る時間） |
| `--extra-topic TOPIC` | | センサー以外に再生する bag のトピック（繰り返し可） |
| `--record-gnss-topic TOPIC` | | そのトピック（`NavSatFix`）を再生に加え、受け取った測位を `gnss.csv` に書く。ノードがそれを使うかは、ノードの `gps_topic` で決まる |
| `--imu-topic TOPIC` | 空（IMU なし） | そのトピック（`sensor_msgs/Imu`）を再生に加え、ノードに `imu_topic:=TOPIC` を渡す。`summary.json` の `imu_diagnostics` と `missing_topics` が埋まる |
| `--save-map-service` | なし | 再生が終わったら、停止の前に `~/save_map` を呼んで応答を待つ。mapping だけ |
| `--save-timeout` | 600 | `save_map` の応答と、停止（SIGINT）後の保存を待つ上限（秒） |
| `--initial-pose X,Y,YAW` | | ノードが active になったら `~/set_initial_pose` を呼ぶ（m・m・rad）。localization だけ |
| `--initial-pose-from-trajectory FILE` | | 地図の `trajectory.csv` から、再生を始める時刻の姿勢を補間して初期姿勢にする。localization だけ |

出力（`--out`）: `map/`、`map_topdown.png`、`summary.json`（keyframes・map_points・bbox・path_length・scan_ms の p50/p95/max・
`aligned_scans_published`・`scans_delivered_to_session` など）、`status.csv`、`online_path.csv`（列は `stamp_sec,x,y,z,yaw`）、`node.log`、
`--record-gnss-topic` を付けたときは `gnss.csv`。

### 受信した測位の記録（`--record-gnss-topic`）

`gnss.csv` の列は `bag_time_ns,latitude,longitude,altitude,status,cov_east,cov_north,cov_up`。時刻は、受け取ったときの再生の時計（`/clock`、bag の時刻）です。
**緯度・経度を含むので、PR・Issue・コミットに貼らない**（このリポジトリは公開予定）。載せるのは差の数値だけ。

### IMU の有り無しを比べる（`--imu-topic`）

`--extra-topic` と `--node-param imu_topic:=...` の両方を書かなくても、`--imu-topic` 1 つで、IMU のトピックの再生とノードへの `imu_topic` の受け渡しをします（片方を忘れて黙って IMU なしになるのを防ぐため）。
同じ記録を、IMU あり・なし（置き換えだけを切る）で再生して比べる例:

```bash
tools/bag_replay/run_replay.sh --bag ~/bags/<IMU の入った bag> --out ~/work/with-imu    --mode mapping --imu-topic /imu/data_raw
tools/bag_replay/run_replay.sh --bag ~/bags/<IMU の入った bag> --out ~/work/without-imu --mode mapping --imu-topic /imu/data_raw \
  --node-param imu_yaw_prediction_enabled:=false
```

- `summary.json` の `imu_diagnostics`: 再生の間に最後に受け取った `/diagnostics` の `localization_node: IMU` の項目（key と値の辞書）。1 つも受け取らなかったとき、`--imu-topic` なしのときは `null`。
- `summary.json` の `missing_topics`: `--imu-topic`・`--extra-topic`・`--record-gnss-topic` で頼んだが記録に無かったトピック（`[]` なら全部あった）。記録のトピックを先に読み、あるものだけを `ros2 bag play --topics` に渡します。無かったものは警告を出して再生は続けます。センサーのトピック（`/velodyne_points` など）は、これまでどおり常に渡します。
- 記録に IMU が無くても、ノードには `imu_topic` を渡します（「有効だが 1 つも届かない」状態を作れます）。そのときは `missing_topics` に入り、`imu_diagnostics` は `null` か、ノードが出した「届いていない」状態になります。
- 記録のトピックは `metadata.yaml` から読みます（記録の形式も `metadata.yaml` の `storage_identifier` から取り、mcap 固定ではありません）。`--imu-topic` なしでも毎回読むので、記録の場所の誤りや壊れた `metadata.yaml` では、トレースバックではなく `bag: cannot read the bag <場所> ...: <理由>` を出して終了コード 1 で止まります。
- `--imu-topic` なしのときの再生するトピックとノードの起動引数は、これまでと同じです（頼んだトピックが記録に無いときの扱いだけ、上のとおり変わりました）。

### 保存の呼び出し（`--save-map-service`）

再生が終わり、ノードが入力を処理し終えたら、停止の前に `~/save_map` を呼びます。応答の `success` と `message` と待った秒数が
`summary.json` の `save_map` に入ります。失敗・時間切れ（`--save-timeout`）のときは終了コード 1 です。
`save_map` のあとの停止（SIGINT）でも保存がもう一度走る（キーフレームが増えていなければ段階 2 から始まる）ので、`--save-timeout` は最大で 2 回分かかります。
付けないと、保存は停止（SIGINT）のときの 1 回だけです（呼ばずに止めたときの地図を見る回に使う）。

### 初期姿勢（localization）

- `--initial-pose X,Y,YAW`: ノードが active になったあと、再生を始める前に `~/set_initial_pose`（`frame_id=map`、yaw 指定あり）を呼びます。
  **受理されなければ、再生を始めずに終了コード 1 で終わります。** 使った姿勢と応答は `summary.json` の `initial_pose` に入ります。
- `--initial-pose-from-trajectory FILE`: `FILE`（地図の `trajectory.csv`）の、再生を始める時刻（bag の開始 ＋ `--start-offset`）の前後の 2 行から、
  位置は線形、向きは近い側の回り方で補間します。その時刻が地図の時刻の範囲の外のとき、前後の行の間が 2 秒を超えるときは、何も起動せずに誤りで終わります。
  使った姿勢（前後の行の時刻も）は `summary.json` の `initial_pose` に入ります。
- 2 つは同時に使えません。どちらも localization だけです（`--save-map-service` は mapping だけ）。

## 評価（`evaluate_replay.py`）

```bash
python3 tools/bag_replay/evaluate_replay.py <--out の directory> [--mode mapping|localization] [--map-directory <読んだ地図の directory>]
```

1 回の再生の出力から、Issue #93 §5.3 の表の数値を求めて表示し、`<--out>/evaluation.json` にも書きます（`--json-output` で変えられる）。
`--mode` の既定は `summary.json` の値。localization では `--map-directory`（読んだ地図の `trajectory.csv` と `map.yaml`）が要ります。
numpy が要ります。

**座標**: lightnav の地図は、GPS 原点からの x = 東・y = 北です。地図に GNSS の原点があるとき、「重ねない差」は、この仮定で GNSS を原点まわりの東・北に直して比べます。

**計算の決まり**（§5.3 の基準はこの計算で決めた。計算を変えるときは、Issue の基準も見直す）

1. 平面への直し方: GNSS の最初の点を基準にする。東 ＝ 経度の差（rad）× cos（基準の緯度）× 6,378,137 m、北 ＝ 緯度の差（rad）× 6,378,137 m。
2. mapping の軌跡: 保存した `map/trajectory.csv`。開始と終了のずれは、最初の行と最後の行の水平の距離。向きの差と、高さの幅（z の最大 − 最小）も出す。
3. GNSS との対応づけ: 軌跡の各行に、時刻が最も近い GNSS の点を対応づける。時刻の差が 1 秒を超える行は使わない（ちょうど 1 秒は使う）。
4. 重ね方: 2 次元の剛体（回転と平行移動だけ。拡大縮小も反転もしない）を最小二乗で求め、重ねたあとの距離の RMS と最大を出す。
5. localization の軌跡: `online_path.csv` を、比べる相手の時刻に線形で補間する（前後 0.2 秒以内に出力があるときだけ）。
   - 地図の `trajectory.csv` の各行との比較: 重ねずに、水平の差の RMS と最大、高さの差の最大。最初に `TRACKING` になる前の行は対象にしない。
   - GNSS の各点との比較（最初に `TRACKING` になったあと。地図の時刻範囲は掛けない）: 4 のとおり重ねた RMS と最大。地図に GNSS の原点があるときは、重ねない差も出す。
6. 状態の割合（`status.csv` のサンプルの数）: 入力が遅れた割合 ＝ `INPUT_STALE` ÷（全体 − `INITIALIZING`）。
   GNSS が使われた割合 ＝ `GPS_ONLY` ÷（全体 − `INITIALIZING` − `INPUT_STALE`）。
   `TRACKING` の割合 ＝ `TRACKING` ÷（最初に `TRACKING` になってからの全体 − `INPUT_STALE`）。
   分母が 0 の割合は `null`。
7. 保存の時間: `node.log` の `map save: <段階> map written <秒> s after the start of the save` の行の秒（すでに出た段階、またはそれより前の段階が来たら次の保存とみなす。段階 2 から始まる保存も 1 回と数える）。

## Mac での再生（評価用の環境 `-e eval`）

Mac では Docker を使わず、評価用の環境に rclpy と ROS が入っています。作業場で `pixi run build` をしてから（`lightnav_msgs` とノードが `install/` に入る）:

```bash
pixi run -e eval bash -c '
  export ROS_DOMAIN_ID=61 DYLD_LIBRARY_PATH="$PIXI_PROJECT_ROOT/install/lib" MPLCONFIGDIR=/tmp/matplotlib-replay
  python tools/bag_replay/replay_session.py --bag ~/bags/<bag> --out ~/work/<run> --mode mapping --rate 2 --start-offset 295 \
    --params-file docker/config/robot.yaml --record-gnss-topic /fix_all --save-map-service
'
```

- ドメインは 42・43・44 を使わない（実機・表示用・試験）。同時に走らせるのは 2 本まで。
- 別の作業場で作った `install` を使うときは、`AMENT_PREFIX_PATH` に足す。
- `--params-file` の `docker/config/*.yaml` は Mac では作業場のパスで渡す（Docker では `/ws/config/...`）。

## 受け入れの再生のやり方（Issue #93 §5.3）

- 最終の設定ファイルをそのまま使い、`--node-param` での上書きはしない。例外は、道具が必ず渡すもの（モード、再生の時計、地図の出力先、読む地図、状態を出す周期）と、Issue の表に書いた上書きだけ。
- どの回も `--record-gnss-topic /fix_all` を付ける（比べる相手の `develop` の版の回は除く）。
- 各条件を **2 回**行い、2 回とも基準を満たすこと。入力が遅れた割合（`evaluate_replay.py` の `input_stale_fraction`）が 8 % を超えた回は数えずにやり直す。数えなかった回も、値と出力を残す。
- mapping の回: `--mode mapping --start-offset 295 --save-map-service`（呼ばずに止める回は `--save-map-service` を付けない）。そのあと `evaluate_replay.py <out>`。
- localization の回: 地図は mapping の 1 回目。`--mode localization --map-file <map.pcd> --map-yaml <map.yaml>` に、初期化は GNSS（付けない）、`--initial-pose 0,0,0`、`--initial-pose-from-trajectory <trajectory.csv>` のどれか。そのあと `evaluate_replay.py <out> --map-directory <地図の directory>`。
- 表の条件（記号・設定・始める位置）は Issue #93 の §5.3 を見る。

`scans_without_aligned_scan_upper_bound` は「記録側が受け取ったスキャン数 − ノードが処理したスキャン数」で、起動直後に使われないスキャンも含む
上限です。記録側の購読も倍率を上げると取りこぼす（`scans_delivered_to_session` が 1 倍 3361、2 倍 2819、4 倍 2634 と減る）ので、
ノードの取りこぼしの正確な数ではなく、目安です。

## Foxglove で見る

`--foxglove` を付けると、ビルド機の `<port>` に Foxglove の WebSocket が立ちます（**認証なし**。信頼できるネットワークでだけ使う）。
Foxglove から `ws://<ビルド機>:<port>` に接続し、3D パネルで固定フレームを `map` にして次を出します。

| トピック | 内容 |
|---|---|
| `/lightnav/localization/map_cloud` | 地図の点群（latched）。mapping はこれまでのスキャンを `map` に置いた**プレビュー**、localization は事前地図 |
| `/lightnav/localization/path` | これまでの軌跡（`nav_msgs/Path`、latched） |
| `/lightnav/localization/aligned_scan` | 現在のスキャン（`map` 座標） |
| `/tf` | `map` → `odom` → `base_footprint` … |

`map_cloud` はオンライン推定の姿勢で置いた点なので、保存される `map.pcd`（キーフレームのバッチ最適化後）とは少し違います。
`localization_node` のパラメータ `visualization_enabled:=false` で `map_cloud` と `path` の出力を止められます（ロボットの設定は変えていません）。

## 上面図だけ作り直す

```bash
python3 tools/bag_replay/render_map.py <map の directory> --output map.png   # numpy・matplotlib が要る（replay イメージにある）
```

マップ点の bbox、軌跡の bbox、経路長（xy）、始点と終点の距離を表示します。

## 試験

```bash
docker run --rm -e MPLCONFIGDIR=/tmp/matplotlib lightnav:replay python3 -m pytest -q -p no:cacheprovider /ws/tools/bag_replay/test_render_map.py /ws/tools/bag_replay/test_replay_session.py \
  /ws/tools/bag_replay/test_replay_arguments.py /ws/tools/bag_replay/test_evaluate_replay.py
```

Mac（評価用の環境。`lightnav_msgs` が `install/` にあること）:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run -e eval bash -c 'export ROS_DOMAIN_ID=61 DYLD_LIBRARY_PATH="$PIXI_PROJECT_ROOT/install/lib"; python -m pytest -q tools/bag_replay'
```
