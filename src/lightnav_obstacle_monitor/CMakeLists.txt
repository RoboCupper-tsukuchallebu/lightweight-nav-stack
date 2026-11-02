cmake_minimum_required(VERSION 3.8)
project(lightnav_obstacle_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lightnav_msgs REQUIRED)

add_library(obstacle_monitor_core
  src/obstacle_grid.cpp
  src/sensor_projection.cpp
  src/frame_processor.cpp)
target_include_directories(obstacle_monitor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(obstacle_monitor_core PUBLIC Eigen3::Eigen)
ament_target_dependencies(obstacle_monitor_core PUBLIC
  sensor_msgs nav_msgs lightnav_msgs)

add_library(obstacle_monitor_node_lib src/obstacle_monitor_node.cpp)
target_link_libraries(obstacle_monitor_node_lib PUBLIC obstacle_monitor_core)
ament_target_dependencies(obstacle_monitor_node_lib PUBLIC
  rclcpp rclcpp_lifecycle tf2 tf2_ros tf2_eigen)

add_executable(obstacle_monitor_node src/main.cpp)
target_link_libraries(obstacle_monitor_node obstacle_monitor_node_lib)

install(TARGETS obstacle_monitor_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # ament_add_gtest rebuilds the vendored gtest, which fails against the conda gtest headers
  # in the pixi environment. Link the installed GTest directly instead.
  find_package(GTest REQUIRED)
  foreach(test_name test_obstacle_grid test_frame_processor)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} obstacle_monitor_core GTest::gtest GTest::gtest_main)
    add_test(NAME ${test_name}
      COMMAND ${test_name}
        --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/${test_name}.gtest.xml)
  endforeach()
endif()

ament_package()
