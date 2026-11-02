"""Pure-Python kinematics and ray casting for the lightnav simulator."""
import math
from dataclasses import dataclass

RING_COUNT = 32
RING_ELEVATION_MIN_RAD = math.radians(-30.67)
RING_ELEVATION_MAX_RAD = math.radians(10.67)
EPSILON = 1e-9


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Wall:
    x1: float
    y1: float
    x2: float
    y2: float


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def apply_command_timeout(linear_mps, angular_radps, age_s, timeout_s):
    """Zero the command when the last /cmd_vel is older than the timeout."""
    if age_s > timeout_s:
        return 0.0, 0.0
    return linear_mps, angular_radps


def integrate_unicycle(pose, linear_mps, angular_radps, delta_s):
    """Midpoint integration of a unicycle model. Returns a new Pose2D."""
    mid_yaw = pose.yaw + angular_radps * delta_s / 2.0
    return Pose2D(
        x=pose.x + linear_mps * math.cos(mid_yaw) * delta_s,
        y=pose.y + linear_mps * math.sin(mid_yaw) * delta_s,
        yaw=wrap_angle(pose.yaw + angular_radps * delta_s),
    )


def offset_point(pose, offset_x, offset_y):
    """World position of a point given in the body frame."""
    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
    return (
        pose.x + cos_yaw * offset_x - sin_yaw * offset_y,
        pose.y + sin_yaw * offset_x + cos_yaw * offset_y,
    )


def world_to_sensor(pose, offset_x, offset_y, world_x, world_y):
    """Express a world point in the sensor frame (yaw aligned with the body)."""
    origin_x, origin_y = offset_point(pose, offset_x, offset_y)
    delta_x, delta_y = world_x - origin_x, world_y - origin_y
    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
    return cos_yaw * delta_x + sin_yaw * delta_y, -sin_yaw * delta_x + cos_yaw * delta_y


def _hit_circle(direction_x, direction_y, circle):
    """Distance along a unit 2D ray from the origin to a circle, or None."""
    projection = circle.x * direction_x + circle.y * direction_y
    squared_miss = circle.x**2 + circle.y**2 - projection**2
    squared_depth = circle.radius**2 - squared_miss
    if squared_depth < 0.0:
        return None
    entry = projection - math.sqrt(squared_depth)
    return entry if entry > EPSILON else None


def _hit_wall(direction_x, direction_y, wall):
    """Distance along a unit 2D ray from the origin to a segment, or None."""
    edge_x, edge_y = wall.x2 - wall.x1, wall.y2 - wall.y1
    denominator = direction_x * edge_y - direction_y * edge_x
    if abs(denominator) < EPSILON:
        return None
    distance = (wall.x1 * edge_y - wall.y1 * edge_x) / denominator
    fraction = (wall.x1 * direction_y - wall.y1 * direction_x) / denominator
    if distance > EPSILON and 0.0 <= fraction <= 1.0:
        return distance
    return None


def cast_ray(azimuth, elevation, circles, walls, sensor_height_m, slope_rad, max_range_m):
    """Range to the nearest surface along a ray in the sensor frame.

    Obstacles are vertical, standing on the ground. The ground is a plane
    rising with `tan(slope_rad)` along the sensor x axis, `sensor_height_m`
    below the sensor origin. Returns None when nothing is hit within range.
    """
    cos_elevation, sin_elevation = math.cos(elevation), math.sin(elevation)
    direction_x, direction_y = math.cos(azimuth), math.sin(azimuth)
    slope = math.tan(slope_rad)
    best = None
    # ponytail: obstacles have unbounded height, add a height field if a low-object test needs it
    for hit_2d in [_hit_circle(direction_x, direction_y, c) for c in circles] + [
        _hit_wall(direction_x, direction_y, w) for w in walls
    ]:
        if hit_2d is None or cos_elevation < EPSILON:
            continue
        distance = hit_2d / cos_elevation
        ground_z = -sensor_height_m + slope * hit_2d * direction_x
        if distance * sin_elevation >= ground_z and (best is None or distance < best):
            best = distance
    slope_term = sin_elevation - slope * cos_elevation * direction_x
    if slope_term < -EPSILON:
        ground_distance = -sensor_height_m / slope_term
        if best is None or ground_distance < best:
            best = ground_distance
    if best is None or best > max_range_m:
        return None
    return best


def ring_elevations(ring_count=RING_COUNT):
    step = (RING_ELEVATION_MAX_RAD - RING_ELEVATION_MIN_RAD) / (ring_count - 1)
    return [RING_ELEVATION_MIN_RAD + ring * step for ring in range(ring_count)]


def scan_ranges(beam_count, circles, walls, max_range_m):
    """2D scan in the horizontal plane (obstacles only); misses become inf."""
    ranges = []
    for beam in range(beam_count):
        azimuth = -math.pi + 2.0 * math.pi * beam / beam_count
        direction_x, direction_y = math.cos(azimuth), math.sin(azimuth)
        hits = [_hit_circle(direction_x, direction_y, c) for c in circles]
        hits += [_hit_wall(direction_x, direction_y, w) for w in walls]
        valid = [hit for hit in hits if hit is not None and hit <= max_range_m]
        ranges.append(min(valid) if valid else math.inf)
    return ranges


def cloud_points(azimuth_count, circles, walls, sensor_height_m, slope_rad, max_range_m):
    """Sensor-frame (x, y, z) points for every ring and azimuth that hits."""
    points = []
    for elevation in ring_elevations():
        for index in range(azimuth_count):
            azimuth = -math.pi + 2.0 * math.pi * index / azimuth_count
            distance = cast_ray(azimuth, elevation, circles, walls, sensor_height_m, slope_rad, max_range_m)
            if distance is None:
                continue
            planar = distance * math.cos(elevation)
            points.append((planar * math.cos(azimuth), planar * math.sin(azimuth), distance * math.sin(elevation)))
    return points
