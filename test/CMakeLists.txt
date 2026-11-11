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

# Compile the real implementation with a deterministic SDK double to exercise
# discovery during Init, filtering, and cleanup without sockets or hardware.
ament_add_gtest(test_sdk_binding
  test_sdk_binding.cpp
  ../src/livox_lidar_driver_impl.cpp
)
target_include_directories(test_sdk_binding PRIVATE
  ../include
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
)
target_link_libraries(test_sdk_binding
  ${PROJECT_NAME}_gtest_main
  ${sensor_msgs_TARGETS}
)

ament_add_gtest(test_driver_config_validation
  test_driver_config_validation.cpp
)
target_link_libraries(test_driver_config_validation
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gtest_main
)
