std_msgs/Header header           # frame_id = base_frame
bool is_stop_zone_occupied
bool is_slow_zone_occupied
bool is_input_stale
string[] stale_sources           # 例: "points", "scan", "tf:velodyne"
uint32 stop_zone_cell_count
uint32 slow_zone_cell_count
float32 nearest_distance_m       # 占有セルが無ければ +inf
float32 processing_time_ms
