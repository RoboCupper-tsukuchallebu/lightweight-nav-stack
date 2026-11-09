# lightnav_safety_gate

`/cmd_vel` に書く最後の lifecycle ノード（`safety_gate_node`、20 Hz）。`active` 中のみ出力する。
優先度 0 の緊急停止（`ObstacleStatus` の stop zone 占有 / `is_input_stale` / 途絶、`VelocityCommand` の途絶 / `is_hard_stop`、自身が `active` でない）のいずれかで、レート制限を迂回して即 Twist 0 を出す。
それ以外は `v_max` / `w_max` に飽和し、加減速をレート制限する（レート制限は 0 でない指令にのみ）。非有限値の指令は捨てて warn し、受信時刻を更新しないので `command_timeout_s` で 0 になる。
`deactivate` 時は Twist 0 を 1 回出して沈黙する。鮮度はヘッダ時刻ではなくノードの受信時刻（steady clock）で判定する。

純関数は `safety_gate_logic.hpp`（発動判定・飽和・レート制限）にあり、gtest で試験する。
処理時間: 1 周期は比較数十回の演算のみで、20 Hz の周期（50 ms）に対し無視できる（専用の実測はしていない）。

実行中に変更した値は保存されず、再起動時に YAML の値に戻る。変更は次の出力周期から反映される。

| パラメータ | 種類・実行中の扱い |
|---|---|
| `v_max`, `w_max` | 正の有限値を固定の天井まで上下に変更可能 |
| `v_max_ceiling`, `w_max_ceiling` | read-only。未指定ならそれぞれの起動値が天井。天井が非有限なら対応する速度上限の変更をすべて拒否（もう一方は独立に判定） |
| `publish_hz`, `command_timeout_s`, `obstacle_status_timeout_s` | read-only |
| `acceleration_max_mps2`, `deceleration_max_mps2`, `angular_acceleration_max_rps2`, `angular_deceleration_max_rps2` | read-only |
| `use_sim_time` | 起動後は変更不可 |

有限の天井を明示する場合、起動値が天井を超える設定は configure に失敗する。

read-only のパラメータは rclcpp が descriptor でコールバックより先に拒否するため、拒否理由は返るが、このノードの WARN ログは出ない。
