cmake_minimum_required(VERSION 3.8)
project(fast_livox_ros_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_library(LIVOX_LIDAR_SDK_LIBRARY
  NAMES livox_lidar_sdk_shared liblivox_lidar_sdk_shared.so
  PATHS /usr/local/lib
  REQUIRED
)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES livox_lidar_api.h livox_lidar_def.h
  REQUIRED
)

add_library(${PROJECT_NAME} SHARED
  src/livox_lidar_driver_impl.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${LIVOX_LIDAR_SDK_LIBRARY}
)

# Node library (component) — separate so it can be used as a component or
# linked into an executable.
add_library(${PROJECT_NAME}_node SHARED
  src/livox_lidar_driver_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}
  rclcpp_components::component
  ${rcl_interfaces_TARGETS}
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_nodes(${PROJECT_NAME}_node
  "fast_livox_ros_driver::LivoxLidarDriverNode"
)

add_executable(${PROJECT_NAME}_node_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node_main PRIVATE
  ${PROJECT_NAME}_node
  rclcpp::rclcpp
)

target_include_directories(${PROJECT_NAME}_node_main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
  # Unit tests (examples — add your tests under test/)
  add_subdirectory(test)
endif()

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_node_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_node)
ament_export_dependencies(rclcpp rclcpp_components rcl_interfaces sensor_msgs)

ament_package()
