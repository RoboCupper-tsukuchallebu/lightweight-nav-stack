# docker（X13 の実行環境）

ロボット PC（X13）で nav-stack を Docker で動かす。ビルドとイメージの入れ方は [CLAUDE.md](../CLAUDE.md) の「Docker」、
compose のコメント（`compose.yaml` の先頭）、`scripts/deploy.sh`。ここには **自己位置推定（`localization` サービス）の
現地手順**を書く（Issue #43）。

公開リポジトリなので、緯度経度・ホスト名・IP・メールアドレスはここにも、PR・Issue・ログの貼り付けにも書かない。

## サービス

| サービス | 中身 | 備考 |
|---|---|---|
| `nav` | bringup の 6 ノード（`safety_gate_node` ほか） | `/cmd_vel` を出す。起動と Start は毎回人が承認する |
| `localization` | `lightnav_localization` の `localization_node` | `/cmd_vel` を出さない。mapping と localization を `LIGHTNAV_LOCALIZATION_MODE` で切り替える |

`localization` は `nav` とは別のサービスで、`localization.launch.py` から起動する（bringup の 6 ノードには入れない）。
パラメータは `config/robot.yaml` の `localization_node` 節（GNSS は `/rtk/fix_all`、Single を外す、1 Hz に間引く、
`map_output_dir: /data/map`）。現地で `robot.yaml` を書き換えない（次の deploy が差分で止まる）。値を変えるときは
`docker/config/robot.yaml` を PR で直す。

## 地図の置き場所

地図は `~/lightnav/data/map/`（コンテナ内 `/data/map/`）の 3 ファイル。

| ファイル | 中身 |
|---|---|
| `map.pcd` | 地図点群 |
| `trajectory.csv` | ティーチ走行の軌跡 |
| `map.yaml` | 原点の緯度経度など。**コミットしない**（`docker/data/` は .gitignore 済み） |

- **mapping の起動は `/data/map` の前の地図を上書きする。** 出力先 `map_output_dir` が localization の読む地図と同じ場所だから。
  残したい地図は先に退避する: `mv ~/lightnav/data/map ~/lightnav/data/map_<日付>`
- `./data` はコンテナの uid（`TSUKUCHALLE_UID/GID`、1000）が書ける所有者にする。`stat -c %u:%g ~/lightnav/data` が
  `1000:1000` でなければ `chown` する（`deploy.sh` はログインユーザーで作るので、uid 1000 とは限らない）。

## 手順（mapping → save_map → localization）

前提: drivers・description・datagnss のコンテナが動いている。`nav` は起動しない。モーターはパッドの Start を人が押すまで
無効、Back で無効化。

1. **準備**。次を記録する: `ros2 topic hz /wheel/odom /velodyne_points /rtk/fix_all`、
   `ros2 run tf2_ros tf2_echo base_footprint velodyne`（と `gnss`）、`free -m`。`.env` の `TSUKUCHALLE_UID/GID` が 1000 で、
   `data` の所有者が `1000:1000`。残したい地図を退避する。
   - `gnss` アンテナの位置（`base_link→gnss`）は仮の値のまま。車軸中心からアンテナ位相中心までの x・y・z を実測して
     tsukuchalle_description の `dimensions.yaml` の `gnss_xyz_above_ground_m` を更新してから、GNSS を使う確認を行う。
2. **mapping を起動**: `LIGHTNAV_LOCALIZATION_MODE=mapping docker compose up -d localization`。
   `/lightnav/localization/status` の `gps_status` が 1（Float）以上になり、`state` が `INITIALIZING` を抜けるまで
   車を止めて待つ（この位置が地図の原点になる）。その後 3 m（`init_travel_m`）をまっすぐ走ると、その向きが初期 yaw になる。
   - **原点のログ行（`mapping origin set at ...`）には緯度経度が出る。** ログを Issue・PR に貼るときは
     `grep -v 'mapping origin set'` で伏せる。
3. **ティーチ走行**: パッドで走る。速度は 0.3 m/s・0.4 rad/s 以下を目安にする（下の「速度の判定」）。`nav` は起動しない。
   `/lightnav/localization/odom` と `status` を Foxglove などで見る。
4. **保存**: 止めて `docker compose exec localization bash -lc 'ros2 service call /localization_node/save_map std_srvs/srv/Trigger'`。
   - 成功すると**記録が止まる**。INFO ログと `status` の `last_error` に `mapping recording stopped after save_map` が出る。
     **これ以後に走った分は地図に入らない。** 続けたいときは mapping を起動し直す（前の地図は上書きされるので退避してから）。
   - `/data/map/` に `map.pcd`・`trajectory.csv`・`map.yaml` の 3 つが揃ったことを確認し、`docker compose stop localization`。
   - `save_map` を忘れて `stop` しても、停止時に 1 回保存される。`save_map` 済みで以後キーフレームが増えていなければ、
     停止時には保存しない（二重の保存と上書きを避ける）。
5. **localization を起動**: `LIGHTNAV_LOCALIZATION_MODE=localization docker compose up -d localization`（既定）。
   静止したまま GNSS（σ ≤ 1.5 m）で位置、静止スキャン 3 個で yaw を決め、`state` が `TRACKING` になることを確認する。
   - **`map.yaml` が無いか原点が無いと、GNSS は WARN ログだけを出して黙って無効になる。** `status` の `gps_status` が 1 以上で、
     `/diagnostics` の `accepted_total` が増えることを確かめる。
   - GNSS が取れない場所では `set_initial_pose` で位置を与える:
     ```bash
     docker compose exec localization bash -lc \
       "ros2 service call /localization_node/set_initial_pose lightnav_msgs/srv/SetInitialPose \
        '{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}}'"
     ```
     yaw の指定のしかたは [`lightnav_localization/README.md`](../src/lightnav_localization/README.md) の「初期位置合わせ」。
6. **確認**（下の「実機での確認」）。
7. **終了**: `docker compose stop localization`。

mode は `up` のときに読まれる。切り替えるときは `stop` してから、新しい mode で `up -d` し直す。

## 停止と `stop_signal: SIGINT`

- `localization` は compose で **`stop_signal: SIGINT`** にしてある。`docker compose stop` の既定の SIGTERM では、Jazzy の
  `ros2 launch` は子のノードを止めずにタスクを cancel するだけで終わり（ログに `using SIGTERM can result in orphaned processes`）、
  tini（PID 1）が終わるとノードはカーネルに SIGKILL される。停止時保存は走らない。SIGINT なら launch が子に SIGINT を送り、
  ノードが `on_shutdown` を通って保存してから終わる。
- **`tty: true`・`stdin_open: true` を付けない。** 付けると launch が noninteractive でなくなり、子に SIGINT を送らない。
  CI（`docker` ジョブ）が `stop_signal`・`stop_grace_period` と、この 2 つが無いことを確かめている。
- launch は SIGINT の `sigterm_timeout`（45 s）後に SIGTERM、さらに `sigkill_timeout`（10 s）後に SIGKILL を送る。
  2 つは**足し算**で、55 s < `stop_grace_period` 60 s に収める。バッチ再最適化は数秒〜十数秒かかる。
- `nav` サービスは `stop_signal` を指定していないので、`docker compose stop nav` は 6 ノードを SIGKILL する。
  本手順では `nav` に手を入れない（別 Issue）。

## drive を再起動したとき

drive コンテナを再起動すると `odom` の原点が 0 に戻る。**localization も再起動する**（`docker compose stop localization` →
`up -d localization`）。

## 実機での確認

4 つの PR が develop に入り、`gnss_xyz_above_ground_m` に実測の x・y・z が入ってから行い、結果を Issue #43 にコメントする。

共通で記録するもの:

- `ros2 topic hz` の各入力
- `status` の `last_scan_time_ms`・`last_match_time_ms`・`position_std_m`
- `free -m` の起動前後と、localization 単体の RSS
- `/diagnostics` の各値（`gate_rejected_total`・`time_gap_rejected_total`・`conflict_window_rejects`・`accepted_total`・
  `gps_status`・`gps_std_m`・`is_gps_conflict`）
- Float が取れるまでの時間
- GNSS の遅延（`/rtk/fix_all` の `header.stamp` と rtkrcv の解の時刻 GPST − 18 s（UTC）の差。一時スクリプトで測り、リポには入れない）。
  あわせて X13 の NTP/chrony の設定を確認する
- `/wheel/odom` の twist の最大 |v|・|ω|
- `docker compose stop localization` の所要時間と、launch ログの `orphaned processes` の有無

ログを貼るときは原点の行を伏せる。

1. **止めたまま**: mapping で Float により `INITIALIZING` を抜ける。localization で静止初期化して `TRACKING`。10 分静止して
   `/lightnav/pose` の漂いが 0.3 m 以内。`last_scan_time_ms` の中央値 < 100 ms、`last_match_time_ms` < 500 ms。
   `INPUT_STALE` がちらつかない。RAM が 6 GiB に収まる。`time_gap_rejected_total` が増え続けない（増えるなら遅延を測り
   `gps_keyframe_max_gap_s` を見直す）。
2. **押して動かす**（モーター無効）: 直線 10 m 往復と 90° 旋回を含めて押し、戻った位置の `/lightnav/pose` が実位置とメジャーで
   0.5 m 以内。`TRACKING` のまま。`is_large_correction` が立った回数を記録する。
3. **パッドで走らせる**（teleop、`nav` は起動しない、Back を非常停止に使う）: ティーチ走行で地図を作り、別の走行で `TRACKING` を維持し、
   始点に戻った `/lightnav/pose` が 0.5 m 以内。旋回で `LOST` にならない。drive の再起動後に localization を再起動して復帰できる。
4. mapping 中に GNSS が途切れる区間（建物の影、または Float が落ちて Single だけになる区間）を 1 回通り、`LOST` にならずに記録が続く。
5. 停止経路:
   - (a) `save_map` を呼ばずに mapping を数十秒動かしてから `docker compose stop localization`。launch のログに
     `using SIGTERM can result in orphaned processes` が**出ない**、ノードのログに停止時の保存が出て 3 ファイルが書かれる、
     `stop` が 60 s 以内に終わる。
   - (b) `save_map` → 静止したまま数十秒待つ → `docker compose stop`。3 ファイルの mtime が `save_map` 時のままで
     （停止時保存が走らない）、`map.yaml` の原点が mapping 開始時のログと同じ。

自動走行（`send_waypoints`、safety_gate、`v_max 0.3`、stop zone）はここでは確認しない。

### 速度の判定

drivers の teleop は、スティックを倒し切ると 0.6 m/s・1.5 rad/s まで出る（2026-11-06 に上げた値）。ここでは teleop の設定を変えず、
`/wheel/odom` の twist を記録して区間を絞る。

- 走行中に `ros2 bag record /wheel/odom` で記録する（localization は twist を読まない。記録専用）。
- 判定（始点に戻った位置、`TRACKING` の維持、`LOST`）に使うのは、twist が **最大 |v| ≤ 0.3 m/s かつ |ω| ≤ 0.4 rad/s の区間だけ**。
  それを超えた区間があれば、その区間は判定から外し、超えた最大値を結果に書く。
