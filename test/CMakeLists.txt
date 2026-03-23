cmake_minimum_required(VERSION 3.8)

# This subdirectory is added only when BUILD_TESTING is enabled from the top-level
# CMakeLists.txt.

find_package(ament_cmake_gtest REQUIRED)

# Provide a single main() for all tests, similar to usb_cam.
add_library(${PROJECT_NAME}_gtest_main
  gtest_main.cpp
)
target_link_libraries(${PROJECT_NAME}_gtest_main PUBLIC
  rclcpp::rclcpp
)

ament_add_gtest(test_ipv4_handle
  test_ipv4_handle.cpp
)
target_link_libraries(test_ipv4_handle
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gtest_main
)

ament_add_gtest(test_driver_config_validation
  test_driver_config_validation.cpp
)
target_link_libraries(test_driver_config_validation
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gtest_main
)
