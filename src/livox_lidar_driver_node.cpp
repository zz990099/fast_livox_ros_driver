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

#include "fast_livox_ros_driver/livox_lidar_driver_node.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace fast_livox_ros_driver
{

// Node workflow
//
// 1) Declare and load parameters.
// 2) Create publishers with configured topics/QoS.
// 3) Create the driver implementation and bind callbacks to publishers.
// 4) Keep spinning (or be composed) while the Livox SDK delivers data.

LivoxLidarDriverNode::LivoxLidarDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fast_livox_ros_driver", options)
{
  // Parameter declaration must happen before reading parameters.
  declare_parameters();
  load_parameters();
  create_publishers();
  start_driver();
}

LivoxLidarDriverNode::~LivoxLidarDriverNode()
{
  if (driver_) {
    driver_->stop();
  }
}

void LivoxLidarDriverNode::declare_parameters()
{
  this->declare_parameter<std::string>("frame_id", "livox_frame");
  this->declare_parameter<std::string>("lidar_ip", "");
  this->declare_parameter<std::string>("config_file_path", "");
  this->declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
  this->declare_parameter<std::string>("imu_topic", "/livox/imu");
  this->declare_parameter<int>("pointcloud_qos_depth", 10);
  this->declare_parameter<int>("imu_qos_depth", 100);
}

void LivoxLidarDriverNode::load_parameters()
{
  frame_id_ = this->get_parameter("frame_id").as_string();
  driver_config_.lidar_ip = this->get_parameter("lidar_ip").as_string();
  driver_config_.config_file_path = this->get_parameter("config_file_path").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  pointcloud_qos_depth_ = this->get_parameter("pointcloud_qos_depth").as_int();
  imu_qos_depth_ = this->get_parameter("imu_qos_depth").as_int();

  if (frame_id_.empty()) {
    throw std::runtime_error("Parameter 'frame_id' must not be empty.");
  }
  if (driver_config_.lidar_ip.empty()) {
    throw std::runtime_error("Parameter 'lidar_ip' must not be empty.");
  }
  if (driver_config_.config_file_path.empty()) {
    throw std::runtime_error("Parameter 'config_file_path' must not be empty.");
  }
  if (pointcloud_topic_.empty() || imu_topic_.empty()) {
    throw std::runtime_error("Output topics must not be empty.");
  }
  if (pointcloud_qos_depth_ <= 0 || imu_qos_depth_ <= 0) {
    throw std::runtime_error("QoS depth parameters must be greater than zero.");
  }
}

void LivoxLidarDriverNode::create_publishers()
{
  // KeepLast depth is used for predictable memory use and typical sensor QoS behavior.
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(pointcloud_qos_depth_))));
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(imu_qos_depth_))));
}

void LivoxLidarDriverNode::start_driver()
{
  // Create the driver and bind callbacks to publishers.
  driver_ = std::make_unique<LivoxLidarDriverImplementation>(
    driver_config_,
    std::bind(&LivoxLidarDriverNode::publish_pointcloud, this, std::placeholders::_1),
    std::bind(&LivoxLidarDriverNode::publish_imu, this, std::placeholders::_1));

  driver_->start();

  // Print a single startup log line with the resolved runtime configuration.
  RCLCPP_INFO(
    this->get_logger(),
    "fast_livox_ros_driver started. lidar_ip='%s' frame_id='%s' "
    "sdk_version='%s' pointcloud_topic='%s' imu_topic='%s'",
    driver_config_.lidar_ip.c_str(),
    frame_id_.c_str(),
    driver_->sdk_version().c_str(),
    pointcloud_topic_.c_str(),
    imu_topic_.c_str());
}

void LivoxLidarDriverNode::publish_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  msg->header.frame_id = frame_id_;
  pointcloud_pub_->publish(*msg);
}

void LivoxLidarDriverNode::publish_imu(sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  msg->header.frame_id = frame_id_;
  imu_pub_->publish(*msg);
}

}  // namespace fast_livox_ros_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fast_livox_ros_driver::LivoxLidarDriverNode)
