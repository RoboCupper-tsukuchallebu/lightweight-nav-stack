cmake_minimum_required(VERSION 3.8)
project(lightnav_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lightnav_msgs REQUIRED)

add_library(lightnav_controller_core
  src/follow_layer.cpp
  src/avoid_layer.cpp
  src/arbitrate.cpp
  src/freshness.cpp)
target_include_directories(lightnav_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lightnav_controller_core PUBLIC cxx_std_17)
target_compile_options(lightnav_controller_core PRIVATE -Wall -Wextra -Werror)

# Link the C++ typesupport targets. On macOS the rosidl_generator_py libraries
# pulled in transitively fail to load ("symbol not found: _PyExc_RuntimeError")
# unless libpython is linked as well.
# The py generator libs reached through rclcpp's own dependencies still need
# libpython resolved at load time, so link it too.
find_package(Python3 REQUIRED COMPONENTS Development)
set(controller_ros_libraries
  Python3::Python
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
  lightnav_msgs::lightnav_msgs__rosidl_typesupport_cpp)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node lightnav_controller_core ${controller_ros_libraries})
target_compile_options(controller_node PRIVATE -Wall -Wextra -Werror)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS lightnav_controller_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # ament_add_gtest builds the vendored gtest sources, which clash with the
  # conda gtest headers in the pixi env, so link the installed GTest instead.
  find_package(ament_cmake_test REQUIRED)
  find_package(GTest REQUIRED)
  foreach(test_name test_follow_layer test_avoid_layer test_arbitrate test_freshness)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} lightnav_controller_core GTest::gtest GTest::gtest_main)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    ament_add_test(${test_name}
      COMMAND $<TARGET_FILE:${test_name}>
        --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${test_name}.gtest.xml
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  endforeach()

  add_executable(test_controller_node test/test_controller_node.cpp)
  target_link_libraries(test_controller_node lightnav_controller_core GTest::gtest GTest::gtest_main
    ${controller_ros_libraries})
  target_compile_options(test_controller_node PRIVATE -Wall -Wextra -Werror)
  ament_add_test(test_controller_node
    COMMAND $<TARGET_FILE:test_controller_node>
      --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_controller_node.gtest.xml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
