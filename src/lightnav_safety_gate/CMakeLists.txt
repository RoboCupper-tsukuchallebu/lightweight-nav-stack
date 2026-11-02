cmake_minimum_required(VERSION 3.8)
project(lightnav_safety_gate)

# Flags are per target: a directory-wide -Werror / standard would also hit the vendored gtest.
set(LIGHTNAV_WARNING_FLAGS -Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lightnav_msgs REQUIRED)

add_library(safety_gate_logic src/safety_gate_logic.cpp)
target_include_directories(safety_gate_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(safety_gate_logic PUBLIC cxx_std_17)
target_compile_options(safety_gate_logic PRIVATE ${LIGHTNAV_WARNING_FLAGS})

add_executable(safety_gate_node src/safety_gate_node.cpp)
target_compile_options(safety_gate_node PRIVATE ${LIGHTNAV_WARNING_FLAGS})
target_link_libraries(safety_gate_node safety_gate_logic)
ament_target_dependencies(safety_gate_node
  rclcpp rclcpp_lifecycle geometry_msgs lightnav_msgs)

install(TARGETS safety_gate_logic safety_gate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # ament_add_gtest builds the vendored gtest sources, which do not match the gtest 1.18
  # headers in the pixi environment. Link the installed GTest and register the test by hand.
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
endif()

ament_package()
