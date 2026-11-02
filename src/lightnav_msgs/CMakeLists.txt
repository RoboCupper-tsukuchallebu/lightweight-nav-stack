cmake_minimum_required(VERSION 3.8)
project(lightnav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/PathWithId.msg"
  "msg/FollowStatus.msg"
  "msg/VelocityCommand.msg"
  "msg/ObstacleStatus.msg"
  "msg/LocalizationStatus.msg"
  "srv/PlanPath.srv"
  "srv/SetInitialPose.srv"
  "action/FollowWaypoints.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs action_msgs
)

ament_package()
