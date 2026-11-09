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

H と footprint は中心 (4.0, 0.3)、半径 0.5 m の円障害物を使う。B は壁の前で停止域に入る場合、0.5 秒以内に停止することを確認する。停止域の手前で回避層が待機する場合は、1 秒間の複数の指令がすべてゼロであること、stuck の報告、障害物除去後の再開を確認する。この待機による合格はユーザー確認待ちであり、停止域に入ったときの停止は safety gate の単体試験でも検証する。いずれも `robot.yaml` の速度と膨張半径 0.3 m を使う。起動時に lifecycle の再要求が必要だった場合は試験を失敗させる。

現時点では H と footprint の安定性は未確認で、H では経路逸脱 4.711 m の失敗が報告されている。原因と修正を調査中であり、起動停滞（#82）による失敗は走行試験の失敗と分けて扱う。

長方形判定の試作では、膨張済み格子上の停止域を走査すると H と footprint で BLOCKED になる走行を確認した。修正は未完成で、マージ不可。試作前の H 6 回は走行成功 5 回、起動停滞（#82）1 回で、4.711 m の逸脱は再現せず原因は未特定。試作後の単独反復は H・footprint が各 3 回中、走行失敗 1 回・起動停滞 2 回、B は成功 1 回・起動停滞 2 回だった。
