// Copyright (c) 2026 zz990099
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "fast_livox_ros_driver/livox_lidar_driver_impl.hpp"

namespace fast_livox_ros_driver
{

/**
 * @brief ROS 2 node wrapper around @ref LivoxLidarDriverImplementation.
 *
 * Parameters:
 * - frame_id (string): frame used for outgoing messages.
 * - config_file_path (string): Livox SDK config file.
 * - pointcloud_topic (string): output pointcloud topic.
 * - imu_topic (string): output IMU topic.
 * - pointcloud_qos_depth (int): QoS depth for pointcloud publisher.
 * - imu_qos_depth (int): QoS depth for IMU publisher.
 */
class LivoxLidarDriverNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node and start the driver.
   *
   * @throws std::runtime_error if required parameters are missing.
   */
  explicit LivoxLidarDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /** @brief Stops the driver (if running). */
  ~LivoxLidarDriverNode() override;

private:
  /** @brief Declare parameters and their default values. */
  void declare_parameters();

  /**
   * @brief Load parameters into member variables and validate them.
   * @throws std::runtime_error on invalid parameter values.
   */
  void load_parameters();

  /** @brief Create publishers for pointcloud and IMU topics. */
  void create_publishers();

  /** @brief Construct and start the underlying driver implementation. */
  void start_driver();

  /** @brief Publish pointcloud message after stamping frame_id. */
  void publish_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /** @brief Publish IMU message after stamping frame_id. */
  void publish_imu(sensor_msgs::msg::Imu::SharedPtr msg);

  /** @brief Frame id applied to outgoing messages. */
  std::string frame_id_;

  /** @brief Output pointcloud topic name. */
  std::string pointcloud_topic_;

  /** @brief Output IMU topic name. */
  std::string imu_topic_;

  /** @brief QoS depth for pointcloud publisher (KeepLast). */
  int pointcloud_qos_depth_{10};

  /** @brief QoS depth for IMU publisher (KeepLast). */
  int imu_qos_depth_{100};

  /** @brief Configuration forwarded to the driver implementation. */
  LivoxDriverConfig driver_config_;

  /** @brief Driver instance, owned by this node. */
  std::unique_ptr<LivoxLidarDriverImplementation> driver_;

  /** @brief PointCloud2 publisher. */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  /** @brief IMU publisher. */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace fast_livox_ros_driver
