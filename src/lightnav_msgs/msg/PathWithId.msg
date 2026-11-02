std_msgs/Header header           # frame_id = global_frame
uint32 session_id                # mission 起動ごとに乱数で生成（0 は無効）
uint32 path_id                   # 0 は「経路なし」。goal ごと・再計画ごとに +1
nav_msgs/Path path               # poses が空 = 停止指示
uint32[] waypoint_indices        # 各 waypoint に対応する poses のインデックス
