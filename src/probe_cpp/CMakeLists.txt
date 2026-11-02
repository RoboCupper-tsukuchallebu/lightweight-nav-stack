cmake_minimum_required(VERSION 3.8)
project(probe_cpp)
find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(GTest REQUIRED)
  add_executable(test_probe test/test_probe.probe_cpp)
  target_link_libraries(test_probe GTest::gtest_main)
  ament_add_test(test_probe
    COMMAND $<TARGET_FILE:test_probe> --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_probe.gtest.xml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
