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
