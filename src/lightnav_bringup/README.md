# lightnav_bringup

ロボット（Docker の `nav` サービス）で lightnav のノードを起動する launch だけのパッケージ。

```
ros2 launch lightnav_bringup robot.launch.py [safety_gate:=true|false] [robot_config:=/config/robot.yaml]
```

起動するノードは 6 つで、すべて lifecycle ノード。launch が起動後に `configure` → `activate` を自動で通す。

| ノード名 | パッケージ |
|---|---|
| `obstacle_monitor` | `lightnav_obstacle_monitor` |
| `pose_adapter_node` | `lightnav_pose_adapter` |
| `planner_node` | `lightnav_planner` |
| `lightnav_mission` | `lightnav_mission` |
| `controller_node` | `lightnav_controller` |
| `safety_gate_node` | `lightnav_safety_gate`（`safety_gate:=true` のときだけ） |

- `safety_gate`（既定 `true`）: `/cmd_vel` に書くのは `safety_gate_node` だけ。`false` にするとこのノードを起動しない（`/cmd_vel` の publisher が無い）。パッドで操縦して `record_waypoints` を使うティーチ走行専用で、この状態で `send_waypoints` をしない。`true` / `false` 以外の値は起動を拒否する。
- `robot_config`: ノードのパラメータ。トップレベルのキーはノード名。既定は `/config/robot.yaml`（compose が `docker/config/robot.yaml` をマウントする）。ファイルが無ければ起動しない。速度上限などの値は `docker/config/robot.yaml` に置き、`safety_gate_node` の上限は yaml に書かず PR #4 の既定のままにする。
- 止めるとき: `ros2 lifecycle set /safety_gate_node deactivate`（Twist 0）→ コンテナを止める。

試験（`test/`）は `docker/config/robot.yaml` を読み、`safety_gate:=false` で `safety_gate_node` が起動せず `/cmd_vel` の publisher が無いことを launch_testing で確かめる。

閉ループ試験用の `sim_closed_loop.launch.py` は sim とスタックを起動し、円障害物の回避と壁による停止・再開を検証する。回避試験は `base_link` と `base_footprint` の両方で実行する。実機には使用しない。実時間の launch_testing は負荷によって変動するため（#32、#40）、回避試験は条件を待ち、タイムアウトを長めに設定している。シナリオ A の 60 秒制限は判定しない。

H と footprint は中心 (4.0, 0.3)、半径 0.5 m の円障害物を使う。B は壁の前で停止域に入る場合、0.5 秒以内に停止することを確認する。停止域の占有または BLOCKED を確認し、障害物除去後の再開を検証する。いずれも `robot.yaml` の速度と膨張半径 0.3 m を使う。起動時に lifecycle の再要求が必要だった場合は試験を失敗させる。

2026-11-10 のビルド機での増分試験: obstacle monitor 58/58、controller 101/101、sim 34/34、bringup 6/7。H と footprint は成功したが、B は走行前に planner が inactive のままになり、lifecycle の再要求が必要だったため失敗した（#82）。planner は単調時計 327627.658 秒で inactive、再要求後の 327632.777 秒に activating へ遷移した。起動停滞の根本原因は未確定。今回の変更後の単独 6 回反復は未実施で、B の 180 秒タイムアウトと footprint の走行中 BLOCKED の原因も未確定。試験ログには各状態受信時の位置、状態、選択円弧、停止域、速度指令、最新の各格子の stamp と占有数を記録する。試験側で観測した最新 stamp の一致は、コントローラの実際の格子選択やフォールバックを示すものではない。
