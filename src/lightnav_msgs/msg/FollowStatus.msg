std_msgs/Header header
uint8 OK=0
uint8 NO_PATH=1
uint8 POSE_STALE=2
uint8 OBSTACLE_STALE=3
uint8 BLOCKED=4
uint8 AT_PATH_END=5
uint8 STUCK=6
uint8 state
uint8 LAYER_FOLLOW=0
uint8 LAYER_AVOID=1
uint8 LAYER_NONE=2               # どの層も主張していない（停止）
uint8 active_layer
uint32 session_id
uint32 path_id
uint32 closest_index
float32 remaining_distance_m     # 最近傍点から経路末端までの弧長
float32 path_deviation_m         # 車体の経路からの距離
float32 commanded_linear_mps     # 調停後の v（不動検知用）
bool is_stuck
