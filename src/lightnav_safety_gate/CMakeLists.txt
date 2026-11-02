cmake_minimum_required(VERSION 3.8)
project(lightnav_safety_gate)

# Flags are per target rather than directory-wide.
set(LIGHTNAV_WARNING_FLAGS -Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lightnav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Link the C++ typesupport targets. On macOS the rosidl_generator_py libraries pulled in
# transitively fail to load ("symbol not found: _PyExc_RuntimeError") unless libpython is
# linked as well (same workaround as lightnav_controller).
find_package(Python3 REQUIRED COMPONENTS Development)
set(safety_gate_ros_libraries
  Python3::Python
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp
  lightnav_msgs::lightnav_msgs__rosidl_typesupport_cpp)

add_library(safety_gate_logic src/safety_gate_logic.cpp)
target_include_directories(safety_gate_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(safety_gate_logic PUBLIC cxx_std_17)
target_compile_options(safety_gate_logic PRIVATE ${LIGHTNAV_WARNING_FLAGS})

add_executable(safety_gate_node src/safety_gate_node.cpp)
target_compile_options(safety_gate_node PRIVATE ${LIGHTNAV_WARNING_FLAGS})
target_link_libraries(safety_gate_node safety_gate_logic ${safety_gate_ros_libraries})

install(TARGETS safety_gate_logic safety_gate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # ament_add_gtest builds the vendored gtest sources, which clash with the conda gtest
  # headers in the pixi env, so link the installed GTest and register the tests by hand.
  find_package(ament_cmake_test REQUIRED)
  find_package(GTest REQUIRED)
  add_executable(test_safety_gate_logic test/test_safety_gate_logic.cpp)
  target_link_libraries(test_safety_gate_logic safety_gate_logic GTest::gtest_main)
  target_compile_features(test_safety_gate_logic PRIVATE cxx_std_17)
  target_compile_options(test_safety_gate_logic PRIVATE ${LIGHTNAV_WARNING_FLAGS})
  ament_add_test(test_safety_gate_logic
    COMMAND $<TARGET_FILE:test_safety_gate_logic>
      --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_safety_gate_logic.gtest.xml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  add_executable(test_safety_gate_node test/test_safety_gate_node.cpp)
  target_link_libraries(test_safety_gate_node safety_gate_logic GTest::gtest_main
    ${safety_gate_ros_libraries})
  target_compile_features(test_safety_gate_node PRIVATE cxx_std_17)
  target_compile_options(test_safety_gate_node PRIVATE ${LIGHTNAV_WARNING_FLAGS})
  ament_add_test(test_safety_gate_node
    COMMAND $<TARGET_FILE:test_safety_gate_node>
      --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_safety_gate_node.gtest.xml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
