std_msgs/Header header
geometry_msgs/Twist twist
bool is_hard_stop                # true: レート制限を迂回して即 0
uint8 source_layer               # FollowStatus の LAYER_*
uint8 reason                     # FollowStatus の state
