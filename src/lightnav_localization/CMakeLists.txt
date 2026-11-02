cmake_minimum_required(VERSION 3.16)
project(lightnav_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(lightnav_localization_core
  src/pcd_io.cpp
  src/map_metadata.cpp
  src/deskew.cpp
  src/downsample.cpp
  src/matcher.cpp)
target_include_directories(lightnav_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Vendored small_gicp (MIT): SYSTEM so that our strict warnings do not apply to it. Private: no
# public header exposes small_gicp types.
target_include_directories(lightnav_localization_core SYSTEM PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/small_gicp/include>)
target_link_libraries(lightnav_localization_core PUBLIC Eigen3::Eigen)
target_compile_options(lightnav_localization_core PRIVATE -Wall -Wextra -Werror)

install(TARGETS lightnav_localization_core EXPORT export_lightnav_localization
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_lightnav_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  # Plain GTest (not ament_add_gtest): the vendored gtest source of ament_cmake_gtest does not
  # compile against the gtest headers in the pixi environment.
  find_package(GTest REQUIRED)
  foreach(test_name pcd_io map_metadata pose_buffer deskew matcher performance)
    add_executable(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} lightnav_localization_core GTest::gtest GTest::gtest_main)
    target_compile_options(test_${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND test_${test_name})
  endforeach()
endif()

ament_package()
