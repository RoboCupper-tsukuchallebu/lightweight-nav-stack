# docker

Docker image, compose and parameters of the nav-stack on the robot PC. The build and the test of the image are in CLAUDE.md ("Docker").

## Indoors, without GNSS (Issue #43 section 12)

`config/localization_indoor.yaml` is the params file of `localization_node` for a room: it listens to no GNSS topic, so
mapping starts at the vehicle pose and the map has no GNSS origin. The node code is the same as outdoors; the params file
is selected with an environment variable of compose.

1. Put `LIGHTNAV_LOCALIZATION_PARAMS=/config/localization_indoor.yaml` in `.env` next to `compose.yaml`. The `localization`
   service of compose reads it as `params_file:=${LIGHTNAV_LOCALIZATION_PARAMS:-/config/robot.yaml}`; without the variable it
   uses `robot.yaml` (outdoors). `compose.yaml` mounts `./config/localization_indoor.yaml` at `/config/localization_indoor.yaml`; `scripts/deploy.sh` puts `config/` on the robot PC.
2. Put the vehicle on the floor mark (a taped cross, the front of the vehicle marked) and start mapping. **Stay still until
   `state` leaves `INITIALIZING`** (3 s after activation). The mark is the origin of the map and the front of the vehicle at
   that moment is +x. Then **drive straight ahead** and drive the course with the pad. Call `save_map` before stopping.
3. Start localization with the same params file, put the vehicle on the mark in the same direction, and call
   `set_initial_pose` with (0, 0, 0) and yaw 0 (`covariance[35]` stays 0). Check that `state` becomes `TRACKING`.
4. Drive, come back to the mark and record the difference of `/lightnav/pose` from (0, 0, 0).

The details (why `gps_topic` is a topic nobody publishes, the `set_initial_pose` call) are in
`src/lightnav_localization/README.md`, section "室内（GNSS 無し）".
