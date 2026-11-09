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

出力（`--out`）: `map/`、`map_topdown.png`、`summary.json`（keyframes・map_points・bbox・path_length・scan_ms の p50/p95/max・
`aligned_scans_published`・`scans_delivered_to_session` など）、`status.csv`、`online_path.csv`、`node.log`。

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
docker run --rm -e MPLCONFIGDIR=/tmp/matplotlib lightnav:replay python3 -m pytest -q -p no:cacheprovider /ws/tools/bag_replay/test_render_map.py /ws/tools/bag_replay/test_replay_session.py
```
