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

#include <livox_lidar_def.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace fast_livox_ros_driver
{

/**
 * @brief Configuration required to initialize the Livox LiDAR SDK.
 */
struct LivoxDriverConfig
{
  /** @brief IPv4 address of the target LiDAR device (e.g. "192.168.1.10"). */
  std::string lidar_ip;

  /**
  * @brief Path to the Livox SDK configuration file.
  *
  * The Livox SDK requires a configuration file to define device/network
  * settings. This driver will pass the path to Livox SDK during
  * initialization.
  */
  std::string config_file_path;
};

/**
 * @brief Core driver implementation wrapping Livox SDK callbacks.
 *
 * This class is responsible for:
 * - Initializing/tearing down the Livox SDK.
 * - Receiving pointcloud/IMU packets via SDK callbacks.
 * - Converting raw packets into ROS 2 message types.
 * - Throttling pointcloud publishing to a fixed interval.
 *
 * The class is intentionally independent from rclcpp::Node to allow reuse and
 * simplify unit testing.
 */
class LivoxLidarDriverImplementation
{
public:
  /** @brief Callback invoked when a complete PointCloud2 message is ready. */
  using PointCloudCallback =
    std::function<void (sensor_msgs::msg::PointCloud2::SharedPtr msg)>;

  /** @brief Callback invoked when an IMU message is decoded from a packet. */
  using ImuCallback =
    std::function<void (sensor_msgs::msg::Imu::SharedPtr msg)>;

  /**
  * @brief Construct the driver.
  *
  * @param config Driver configuration.
  * @param pointcloud_callback Non-null callback for pointcloud publication.
  * @param imu_callback Optional IMU callback (can be empty).
  *
  * @throws std::runtime_error If required configuration is missing.
  */
  explicit LivoxLidarDriverImplementation(
    LivoxDriverConfig config,
    PointCloudCallback pointcloud_callback,
    ImuCallback imu_callback);

  /** @brief Destructor will stop/uninitialize the SDK if running. */
  ~LivoxLidarDriverImplementation();

  LivoxLidarDriverImplementation(const LivoxLidarDriverImplementation &) = delete;
  LivoxLidarDriverImplementation & operator=(const LivoxLidarDriverImplementation &) = delete;

  /**
   * @brief Initialize the Livox SDK and register packet callbacks.
   *
   * This method is idempotent.
   */
  void start();

  /**
   * @brief Uninitialize the Livox SDK and reset internal state.
   *
   * This method is idempotent.
   */
  void stop();

  /** @brief Whether the SDK has been successfully initialized. */
  bool is_running() const noexcept;

  /** @brief Livox SDK version string ("major.minor.patch"). */
  const std::string & sdk_version() const noexcept;

private:
  // NOTE: Some helper methods are made protected (see below) to allow unit
  // testing without requiring hardware access.

  struct CloudPointXYZIT
  {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    double timestamp;
  };

  struct RawImuPoint
  {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
  };

  /**
   * @brief Static SDK entry point for pointcloud packets.
   *
   * Forwards to @ref handle_point_cloud_packet using @p client_data.
   */
  static void GlobalPointCloudCallback(
    uint32_t handle,
    uint8_t dev_type,
    LivoxLidarEthernetPacket * data,
    void * client_data);

  /**
   * @brief Static SDK entry point for IMU packets.
   *
   * Forwards to @ref handle_imu_packet using @p client_data.
   */
  static void GlobalImuCallback(
    uint32_t handle,
    uint8_t dev_type,
    LivoxLidarEthernetPacket * data,
    void * client_data);

  /**
   * @brief Handle a single ethernet packet carrying pointcloud samples.
   *
   * The packet is validated, appended to the current cloud, and a publish may
   * be triggered if the throttling interval has elapsed.
   */
  void handle_point_cloud_packet(
    uint32_t handle,
    uint8_t dev_type,
    LivoxLidarEthernetPacket * data);

  /**
   * @brief Handle a single ethernet packet carrying IMU samples.
   *
   * Converts raw IMU samples to a ROS message and forwards to imu callback.
   */
  void handle_imu_packet(
    uint32_t handle,
    uint8_t dev_type,
    LivoxLidarEthernetPacket * data);

  /**
   * @brief Publish the current PointCloud2 when (time + data) conditions are met.
   */
  void publish_pointcloud_if_ready();

protected:
  /**
   * @brief Encode an IPv4 address into a 32-bit SDK handle.
   *
   * The current implementation packs octets in little-endian order:
   * handle = o1<<0 | o2<<8 | o3<<16 | o4<<24.
   *
   * @param ip IPv4 string.
   * @return 32-bit handle.
   * @throws std::runtime_error if @p ip is not a valid IPv4 string.
   */
  uint32_t get_handle_from_ip(const std::string & ip) const;

private:
  /** @brief Compute data payload size (bytes) of a Livox ethernet packet. */
  std::size_t get_eth_packet_byte_size(const LivoxLidarEthernetPacket * data) const;

  /**
   * @brief Generate a ROS timestamp in nanoseconds for the packet.
   *
   * Uses current system time and compensates the packet time interval.
   */
  uint64_t get_eth_packet_timestamp(uint32_t packet_time_interval_100ns) const;

  /** @brief Create a pre-configured PointCloud2 message template with fixed fields. */
  sensor_msgs::msg::PointCloud2::SharedPtr create_pointcloud_msg() const;

  /**
   * @brief Append points from a raw packet into an existing PointCloud2 message.
   *
   * @throws std::runtime_error for unsupported data_type or capacity overflow.
   */
  void append_pointcloud_from_packet(
    const unsigned char * data,
    int data_type,
    uint32_t point_count,
    uint64_t pkt_timestamp,
    uint32_t point_time_interval_ns,
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg) const;

  /** @brief Decode raw IMU bytes into a ROS IMU message. */
  void fill_imu_message(
    const unsigned char * data,
    const sensor_msgs::msg::Imu::SharedPtr & msg) const;

  /** @brief User-provided config, validated in the constructor. */
  LivoxDriverConfig config_;

  /** @brief Callback to deliver aggregated pointcloud messages. */
  PointCloudCallback pointcloud_callback_;

  /** @brief Optional callback to deliver decoded IMU messages. */
  ImuCallback imu_callback_;

  /** @brief Cached SDK handle derived from lidar_ip; used to filter packets. */
  uint32_t self_handle_{0};

  /** @brief SDK init state; guards start/stop idempotency. */
  bool sdk_initialized_{false};

  /** @brief Cached SDK version string for logging and diagnostics. */
  std::string sdk_version_;

  /** @brief Aggregated pointcloud message under construction. */
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_;

  /** @brief Wall-clock time of last publication for throttling. */
  std::chrono::system_clock::time_point last_publish_time_{};
};

}  // namespace fast_livox_ros_driver
