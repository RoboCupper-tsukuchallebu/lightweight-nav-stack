# bag_replay

保存した rosbag を `localization_node` に流して mapping（または localization）する道具です（Issue #43）。ビルド機で使います。
実機（ドメイン 42）・GUI（43）・試験（44）には繋がらず、ドメインは **61** で走ります。

- 再生は `ros2 bag play --clock --rate R --start-offset S`、ノードは `use_sim_time:=true` で動くので、実時間より速い再生（`--rate 2`、`4`）でも
  地図を作れます（`docker/config/*.yaml` は編集しません。上書きは `-p` で渡します）。
- 再生するのは `/velodyne_points` `/wheel/odom` `/tf` `/tf_static` だけです。`--start-offset` で飛ばした `/tf_static` は bag の先頭から読んで出し直します。
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
| `--hold-seconds` | 0 | 再生後、保存の前にこの秒数だけノードと bridge を生かす（Foxglove で見る時間） |

出力（`--out`）: `map/`、`map_topdown.png`、`summary.json`（keyframes・map_points・bbox・path_length・scan_ms の p50/p95/max・
`aligned_scans_published`・`scans_delivered_to_session` など）、`status.csv`、`online_path.csv`、`node.log`。

`scans_without_aligned_scan_upper_bound` は「bag から届いたスキャン数 − ノードが処理したスキャン数」で、起動直後に使われないスキャンも含む
上限です。倍率を上げたときに増えていれば、ノードがスキャンを取りこぼしています。

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
docker run --rm --entrypoint python3 lightnav:replay -m pytest -q /ws/tools/bag_replay/test_render_map.py
```
