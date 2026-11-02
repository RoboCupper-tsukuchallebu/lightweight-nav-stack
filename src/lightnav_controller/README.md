# lightnav_controller

`controller_node` (lifecycle) turns the accepted path, `/lightnav/pose` and `/lightnav/obstacle_status` into `/lightnav/cmd_vel_raw` (`VelocityCommand`) and `/lightnav/follow_status` at 20 Hz while active. The follow layer (pure pursuit, priority 2) and `arbitrate` (priority 1 avoid over priority 2 follow) are pure C++ in `lightnav_controller_core`. The avoid layer is only an interface plus `NullAvoidLayer` (never asserts) until PR4.

Measured: follow + arbitrate for one cycle on a 200 m path takes about 0.0003 ms (macOS arm64, Release). The 2 ms budget including 15 candidate arcs is re-measured in PR4.
