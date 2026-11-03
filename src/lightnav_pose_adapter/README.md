# lightnav_pose_adapter

Converts a `nav_msgs/Odometry` into a 2D `base_link` pose (`/lightnav/pose`, `PoseStamped`, `map`).
The output is `T_map_odom * T_odom_body * T_body_base` projected to (x, y, yaw); the input stamp is kept.

Inputs are dropped (warning throttled to 1 s) when `frame_id` differs from a non-empty `expected_frame_id`,
when any pose value is non-finite, or when the x or y diagonal of `pose.covariance` exceeds
`max_position_variance_m2`. Invalid parameters make `on_configure` fail. The node is a lifecycle node.

## Parameters

| name | default | meaning |
|---|---|---|
| `odom_topic` | `/lightnav/localization/odom` | input Odometry |
| `output_topic` | `/lightnav/pose` | output PoseStamped |
| `expected_frame_id` | `map` | empty disables the check |
| `body_to_base_xyz_rpy` | `[0,0,0,0,0,0]` | T_body_base (x, y, z, roll, pitch, yaw) |
| `map_from_odom_xy_yaw` | `[0,0,0]` | T_map_odom (x, y, yaw) |
| `max_position_variance_m2` | `4.0` | covariance limit |

## YAML examples

Via lightnav_localization (default; identity, the localization output is already `map` to `base_link`):

```yaml
pose_adapter_node:
  ros__parameters:
    odom_topic: /lightnav/localization/odom
    expected_frame_id: map
```

Simulator (`/sim/odom`, frame `world`, reference point offset from `base_link` by `sensor_offset_xyz`;
`body_to_base` is the inverse offset, here for `[0.1, 0, 0.3]`):

```yaml
pose_adapter_node:
  ros__parameters:
    odom_topic: /sim/odom
    expected_frame_id: world
    body_to_base_xyz_rpy: [-0.1, 0.0, -0.3, 0.0, 0.0, 0.0]
```

Direct `/wheel/odom` (bench test without map or GPS; the `odom` origin is treated as `map`):

```yaml
pose_adapter_node:
  ros__parameters:
    odom_topic: /wheel/odom
    expected_frame_id: odom
```
