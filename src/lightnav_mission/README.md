# send_waypoints の中断

`send_waypoints` は SIGINT をフラグで受け取り、0.1 秒以下の spin ごとに確認します。
送信前の Ctrl-C では goal を送信せず、送信後は受理応答を最大 5 秒、
cancel 応答を最大 2 秒待って終了コード 1 で終了します。
確認中の追加の Ctrl-C も例外を発生させず、同じ期限まで確認を続けます。

cancel 応答の `goals_canceling` が空の場合や応答が期限に間に合わない場合は、
ロボットが走行中の可能性を stderr に表示します。
「goal を cancel した」は cancel 応答で対象が確認できたことを示し、実機停止の確認ではありません。

通信を伴う検証は `test/test_send_waypoints_interrupt.py` の 3 件で行います。
ROS_DOMAIN_ID=48 を使用し、実機用の domain 42 は使用しません。

### 実行時パラメータ

到着半径、通過判定、計画タイムアウト、移動検知と再計画の設定は実行中に変更でき、次の tick から使われます。計画の締め切りには計画開始時の `plan_timeout_s` を使い、姿勢の欠落中に期限を延長するときも同じ値を使います。`pose_timeout_s` は起動時の値を上限として、正の有限値にだけ変更できます。トピック名と `tick_hz`、`use_sim_time` は実行時には変更できません。変更は YAML に保存されず、再起動すると起動時の値に戻ります。正であるべき値や非負であるべき値が有限でない場合も configure は失敗します。


| 種類 | パラメータ | 既定値 | 実行時の変更 |
|---|---|---:|---|
| tuning | `default_arrival_radius_m` | 1.0 | 正の有限値 |
| tuning | `final_arrival_radius_m` | 0.5 | 正の有限値 |
| tuning | `pass_index_margin` | 5 | 0 以上の整数 |
| tuning | `plan_timeout_s` | 5.0 | 正の有限値 |
| tuning | `v_progress_min` | 0.1 | 0 以上の有限値 |
| tuning | `no_motion_timeout_s` | 5.0 | 正の有限値 |
| tuning | `no_motion_distance_m` | 0.1 | 正の有限値 |
| tuning | `replan_on_large_correction` | `true` | 真偽値 |
| safety | `pose_timeout_s` | 0.5 | 正の有限値で、起動時の値以下（↓） |
| structural (read_only) | `pose_topic` | `/lightnav/pose` | 不可 |
| structural (read_only) | `localization_status_topic` | `/lightnav/localization/status` | 不可 |
| structural (read_only) | `tick_hz` | 10.0 | 不可 |
| structural | `use_sim_time` | `false` | コンストラクタ後は不可 |

安全パラメータは起動時の値が有限でなければ変更をすべて拒否します。

## mapping の走行から waypoint を作る（`trajectory_to_waypoints`）

mapping（`lightnav_localization` の `mode: mapping`）が保存する `trajectory.csv` を間引いて、`send_waypoints` が読める
`waypoints.yaml` を書きます（Issue #86）。mapping の手動走行がそのままティーチ走行になり、`record_waypoints` と
`refine_waypoints` を別に走らせる必要がありません。ROS には依存しません。

```
ros2 run lightnav_mission trajectory_to_waypoints <trajectory.csv> [--spacing-m 2.0] [--output waypoints.yaml] [--overwrite]
```

- 間引きは `record_waypoints` と同じ規則です。最初の点は必ず使い、前に使った点から `--spacing-m` 以上離れるたびに使い、最後の点（停止位置）は必ず使います。`trajectory.csv` の行はキーフレームごと（既定で約 1 m または 1 秒）なので、点の間隔は `--spacing-m` より少し長くなります。
- 出力は `frame_id: map`、`recorded_with: trajectory`、`source_trajectory`（入力のパス）と、各点の `x`・`y`・`yaw`・`stamp` です。座標は、その `trajectory.csv` と一緒に保存された地図のものです。mapping をやり直したら、waypoint も作り直してください。
- 出力が既にあると書かずに終わります（上書きは `--overwrite`）。出力先が入力と同じファイルなら、`--overwrite` を付けても書きません。
- 終了コードは 0（書き出した）か 1（入力不正: 読めない、列が無い、データ行が無い、非有限値、引数の誤り、出力の問題）です。
- 走った軌跡をそのまま使うので、切り返しや後退をした場所も waypoint になります。走らせる前に中身を確かめてください。
