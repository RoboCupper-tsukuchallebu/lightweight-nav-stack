std_msgs/Header header
uint8 INITIALIZING=0
uint8 TRACKING=1
uint8 GPS_ONLY=2
uint8 ODOM_ONLY=3
uint8 LOST=4
uint8 INPUT_STALE=5
uint8 state
uint8 MODE_LOCALIZATION=0
uint8 MODE_MAPPING=1
uint8 mode
bool is_large_correction        # 未適用の補正が閾値超え → mission は再計画
bool is_gps_conflict            # 地図照合と GPS が連続して矛盾
float32 pending_correction_m
float32 last_inlier_ratio       # 地図照合
float32 last_lidar_odom_inlier_ratio
float32 seconds_since_last_match
float32 odom_only_distance_m
uint8 gps_status                # NavSatStatus.status の写し（-1 → 255）
float32 gps_std_m               # 採用中の GPS 標準偏差
float32 last_scan_time_ms
float32 last_match_time_ms
float64[3] position_std_m       # x, y, yaw(rad)
string last_error
