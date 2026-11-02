cmake_minimum_required(VERSION 3.8)
project(lightnav_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lightnav_msgs REQUIRED)

add_library(lightnav_controller_core
  src/follow_layer.cpp
  src/arbitrate.cpp)
target_include_directories(lightnav_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lightnav_controller_core PUBLIC cxx_std_17)
target_compile_options(lightnav_controller_core PRIVATE -Wall -Wextra -Werror)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node lightnav_controller_core)
target_compile_options(controller_node PRIVATE -Wall -Wextra -Werror)
ament_target_dependencies(controller_node
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs lightnav_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS lightnav_controller_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # ament_add_gtest builds the vendored gtest sources, which clash with the
  # conda gtest headers in the pixi env, so link the installed GTest instead.
  find_package(ament_cmake_test REQUIRED)
  find_package(GTest REQUIRED)
  foreach(test_name test_follow_layer test_arbitrate)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} lightnav_controller_core GTest::gtest GTest::gtest_main)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    ament_add_test(${test_name}
      COMMAND $<TARGET_FILE:${test_name}>
        --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${test_name}.gtest.xml
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  endforeach()
endif()

ament_package()
