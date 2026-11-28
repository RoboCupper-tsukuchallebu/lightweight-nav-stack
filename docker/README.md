# docker

Docker image, compose and parameters of the nav-stack on the robot PC. The build and the test of the image are in CLAUDE.md ("Docker").

## Fast DDS shared memory

All ROS containers across the robot's Compose projects use `ipc: host`, `pid: host`, and the same uid and gid
(`TSUKUCHALLE_UID` / `TSUKUCHALLE_GID`, 1000 on the robot). With different IPC namespaces, discovery can succeed while
data silently fails to arrive. Sharing PID and uid is also part of the operating configuration decided in
[drivers #33](https://github.com/RoboCupper-tsukuchallebu/tsukuchalle_drivers/issues/33); the PID mismatch itself was
not confirmed as the cause. The existing SHM and loopback UDP profile stays in use without Data Sharing.

Do not remove anything from `/dev/shm` while ROS is running. After stopping all ROS containers, use the cleanup script
being added to drivers in its separate PR: `ros2/tools/dds_shm_clean.sh` ([drivers #33](https://github.com/RoboCupper-tsukuchallebu/tsukuchalle_drivers/issues/33),
[PR #34](https://github.com/RoboCupper-tsukuchallebu/tsukuchalle_drivers/pull/34)); it is not yet in develop. The robot has
multiple Compose projects, so the script checks `docker ps`, image names and Compose project labels for running containers
before invoking `fastdds shm clean`. It conservatively refuses cleanup if any Compose container is running. Stop ROS
processes outside containers too, and do not start ROS during cleanup.

## Outdoors, with GNSS: mapping and saving the map (Issue #93)

`config/robot.yaml` is the params file of `localization_node` outdoors (the default of the `localization` service). For
mapping it listens to `/fix_all` and uses the single-point solution too (`gps_min_status: 0`). The values and the reasons
are in `src/lightnav_localization/README.md`, section "設定ファイルの値（`robot.yaml` と室内用）".

1. Move the previous map away (starting mapping overwrites `./data/map`), then start mapping:
   `LIGHTNAV_LOCALIZATION_MODE=mapping docker compose up -d localization`.
2. **Right after power-on, mapping waits for the map origin, for a few minutes.** A fix is used for the origin only while the
   error that the receiver reports for it is at most 10 m (`init_gps_reported_std_max_m`; about the first 3.5 minutes after
   power-on in the recording of Issue #93). While it waits, `last_error` of `/lightnav/localization/status` says
   `waiting for a GPS fix with reported std <= 10.0 m (latest <value> m)`. **Start driving after that wait has disappeared
   from `last_error`.** The part driven during the wait does not enter the map.
3. Drive the course. To build a map with loops closed, **stop the vehicle first, then call `save_map` before stopping the
   container**:

   ```bash
   docker compose exec localization bash -lc \
     'ros2 service call /localization_node/save_map std_srvs/srv/Trigger'
   ```

   Loop closure runs only inside this call and takes minutes (a little over 3 minutes for a 37-minute run on a Mac); the
   node does nothing else meanwhile (no pose, no state), and the response returns when it ends. Then
   `docker compose stop localization`: the map that `save_map` wrote is kept, the stop does not save again.
4. **Stopping without `save_map` gives a map without loop closure.** The save at stop (`docker compose stop`, SIGINT to
   `ros2 launch`) goes through stages 1 to 3 without loop closure and has a **limit of 45 seconds** (`sigterm_timeout:=45`,
   then `sigkill_timeout:=10`; `stop_grace_period: 60s` must stay above their sum). If the limit passes, the node is killed
   and the map of the last finished stage stays.
5. **Stopping from the service screen of the viewer (`tsukuchalle_viewer`) does not call `save_map`.** It is handled like
   `docker compose stop`, so the map has no loop closure. When a map with loops closed is needed, call `save_map` before
   stopping.

Known limits (both in the section "既知の限界" of `src/lightnav_localization/README.md`): a fix without a reported error
(error type "unknown") is not refused by `init_gps_reported_std_max_m`, so if the driver stops reporting errors the check
silently stops working; and there is no params file for outdoors where GNSS cannot be received (`robot.yaml` does not start
mapping without GNSS, and the indoor file has no loop closure).

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
   The indoor params file does not run loop closure (it does not set `loop_closure_enabled`), so `save_map` here writes the
   map without loop closure. Stopping from the service screen of the viewer does not call `save_map` (see above).
3. Start localization with the same params file, put the vehicle on the mark in the same direction, and call
   `set_initial_pose` with (0, 0, 0) and yaw 0 (`covariance[35]` stays 0). Check that `state` becomes `TRACKING`.
4. Drive, come back to the mark and record the difference of `/lightnav/pose` from (0, 0, 0).

The details (why `gps_topic` is a topic nobody publishes, the `set_initial_pose` call) are in
`src/lightnav_localization/README.md`, section "室内（GNSS 無し）".
