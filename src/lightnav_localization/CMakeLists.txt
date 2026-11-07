cmake_minimum_required(VERSION 3.16)
project(lightnav_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(lightnav_msgs REQUIRED)
# macOS: rosidl_generator_py libraries reached through rclcpp need libpython at load time.
find_package(Python3 REQUIRED COMPONENTS Development)

add_library(lightnav_localization_core
  src/pcd_io.cpp
  src/map_metadata.cpp
  src/deskew.cpp
  src/downsample.cpp
  src/matcher.cpp
  src/pose_graph.cpp
  src/gps_input.cpp
  src/lidar_odometry.cpp
  src/initial_alignment.cpp
  src/point_cloud_extraction.cpp
  src/localizer.cpp)
target_include_directories(lightnav_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Vendored small_gicp (MIT): SYSTEM so that our strict warnings do not apply to it. Private: no
# public header exposes small_gicp types.
target_include_directories(lightnav_localization_core SYSTEM PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/small_gicp/include>)
# GTSAM / GeographicLib: SYSTEM includes so their headers do not trip -Werror. Private: no public
# header exposes their types.
target_include_directories(lightnav_localization_core SYSTEM PRIVATE ${GTSAM_INCLUDE_DIR} ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(lightnav_localization_core PUBLIC Eigen3::Eigen
  PRIVATE gtsam gtsam_unstable ${GeographicLib_LIBRARIES})
target_compile_options(lightnav_localization_core PRIVATE -Wall -Wextra -Werror)

add_library(lightnav_localization_node_lib src/localization_node.cpp)
target_include_directories(lightnav_localization_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Link the C++ typesupport targets explicitly. ament_target_dependencies would also pull the
# rosidl_generator_py libraries, which need Python symbols that plain C++ executables do not have.
target_link_libraries(lightnav_localization_node_lib PUBLIC
  lightnav_localization_core
  Python3::Python
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  lightnav_msgs::lightnav_msgs__rosidl_typesupport_cpp)
target_compile_options(lightnav_localization_node_lib PRIVATE -Wall -Wextra -Werror)

add_executable(localization_node src/localization_node_main.cpp)
target_link_libraries(localization_node lightnav_localization_node_lib)
target_compile_options(localization_node PRIVATE -Wall -Wextra -Werror)

install(TARGETS localization_node lightnav_localization_node_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS lightnav_localization_core EXPORT export_lightnav_localization
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_lightnav_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 GTSAM GeographicLib lightnav_msgs rclcpp rclcpp_lifecycle
  nav_msgs sensor_msgs geometry_msgs std_msgs std_srvs tf2 tf2_ros tf2_eigen)

if(BUILD_TESTING)
  # Plain GTest (not ament_add_gtest): the vendored gtest source of ament_cmake_gtest does not
  # compile against the gtest headers in the pixi environment.
  find_package(GTest REQUIRED)
  foreach(test_name pcd_io map_metadata pose_buffer deskew matcher performance pose_graph gps_input
                    corrector state_machine lidar_odometry localizer match_information
                    performance_large initial_alignment point_cloud_extraction)
    add_executable(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} lightnav_localization_core GTest::gtest GTest::gtest_main)
    target_compile_options(test_${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND test_${test_name})
  endforeach()

  add_executable(test_localization_node test/test_localization_node.cpp)
  target_link_libraries(test_localization_node
    lightnav_localization_node_lib GTest::gtest GTest::gtest_main)
  target_compile_options(test_localization_node PRIVATE -Wall -Wextra -Werror)
  ament_add_test(test_localization_node
    COMMAND $<TARGET_FILE:test_localization_node>
      --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_localization_node.gtest.xml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pose_adapter_acceptance test/test_pose_adapter_acceptance.py)
endif()

ament_package()
