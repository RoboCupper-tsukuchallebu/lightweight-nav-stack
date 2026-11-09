# lightnav_safety_gate

汎用出力 `/cmd_vel` に書く最後の lifecycle ノード（`safety_gate_node`、20 Hz）。`active` 中のみ出力する。
優先度 0 の緊急停止（`ObstacleStatus` の stop zone 占有 / `is_input_stale` / 途絶、`VelocityCommand` の途絶 / `is_hard_stop`、自身が `active` でない）のいずれかで、レート制限を迂回して即 Twist 0 を出す。
それ以外は `v_max` / `w_max` に飽和し、加減速をレート制限する（レート制限は 0 でない指令にのみ）。非有限値の指令は捨てて warn し、受信時刻を更新しないので `command_timeout_s` で 0 になる。
`deactivate` 時は Twist 0 を 1 回出して沈黙する。鮮度はヘッダ時刻ではなくノードの受信時刻（steady clock）で判定する。

純関数は `safety_gate_logic.hpp`（発動判定・飽和・レート制限）にあり、gtest で試験する。
処理時間: 1 周期は比較数十回の演算のみで、20 Hz の周期（50 ms）に対し無視できる（専用の実測はしていない）。

robot.launch.py は汎用出力 `/cmd_vel` を `/cmd_vel_nav` に remap する。駆動系の `/cmd_vel` は drivers の twist_mux が出力する。
