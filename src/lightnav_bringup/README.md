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
