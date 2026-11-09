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

- `safety_gate`（既定 `true`）: `/cmd_vel_nav` に書くのは `safety_gate_node` だけ。`false` にするとこのノードを起動しない（`/cmd_vel_nav` の publisher が無い）。パッドで操縦して `record_waypoints` を使うティーチ走行専用で、この状態で `send_waypoints` をしない。`true` / `false` 以外の値は起動を拒否する。
- `robot_config`: ノードのパラメータ。トップレベルのキーはノード名。既定は `/config/robot.yaml`（compose が `docker/config/robot.yaml` をマウントする）。ファイルが無ければ起動しない。速度上限などの値は `docker/config/robot.yaml` に置き、`safety_gate_node` の上限は yaml に書かず PR #4 の既定のままにする。
- 止めるとき: `ros2 lifecycle set /safety_gate_node deactivate`（Twist 0）→ コンテナを止める。

試験（`test/`）は `docker/config/robot.yaml` を読み、`safety_gate:=false` で `safety_gate_node` が起動せず `/cmd_vel_nav` の publisher が無いことを launch_testing で確かめる。

`/cmd_vel_nav` は drivers の twist_mux に入力し、手動操縦 `/cmd_vel_teleop` を優先する。駆動系への `/cmd_vel` は drivers が出力する（drivers #29）。
