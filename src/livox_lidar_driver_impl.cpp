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

#include "fast_livox_ros_driver/livox_lidar_driver_impl.hpp"

#include <livox_lidar_api.h>

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace fast_livox_ros_driver
{

// Implementation notes
//
// 1) start(): initializes the Livox SDK with a config file and registers global
//    callbacks (C style) that forward into this instance.
// 2) SDK callbacks deliver ethernet packets (pointcloud / IMU). We validate the
//    packet pointers and filter by the handle bound by the device info callback.
// 3) Pointcloud packets are appended into a preconfigured PointCloud2 message.
//    Publishing is throttled by kPointCloudPublishInterval to reduce overhead.
// 4) IMU packets are converted to sensor_msgs::msg::Imu and forwarded
//    immediately.

namespace
{

// SDK callbacks and initialization are process-global. Serialize ownership.
std::mutex sdk_lifecycle_mutex;
LivoxLidarDriverImplementation * sdk_owner = nullptr;

// Publish throttling interval. The driver aggregates points into a PointCloud2
// and publishes at this interval to balance latency and CPU load.
constexpr auto kPointCloudPublishInterval = std::chrono::milliseconds{100};

// Upper bound for internal point buffer to avoid unbounded growth.
constexpr std::size_t kMaxPointCloudCapacity = 50000U;
constexpr uint64_t kHundredNanosecondsToNanoseconds = 100ULL;
constexpr uint32_t kPointCloudHeight = 1U;
constexpr uint32_t kPointCloudFieldCount = 6U;
constexpr uint32_t kSingleFieldElementCount = 1U;
constexpr float kMillimetersToMeters = 0.001F;
constexpr float kStandardGravity = 9.80665F;
constexpr std::size_t kXFieldIndex = 0U;
constexpr std::size_t kYFieldIndex = 1U;
constexpr std::size_t kZFieldIndex = 2U;
constexpr std::size_t kIntensityFieldIndex = 3U;
constexpr std::size_t kTagFieldIndex = 4U;
constexpr std::size_t kTimestampFieldIndex = 5U;

}  // namespace

LivoxLidarDriverImplementation::LivoxLidarDriverImplementation(
  LivoxDriverConfig config,
  PointCloudCallback pointcloud_callback,
  ImuCallback imu_callback)
: config_(std::move(config)),
  pointcloud_callback_(std::move(pointcloud_callback)),
  imu_callback_(std::move(imu_callback))
{
  if (config_.config_file_path.empty()) {
    throw std::runtime_error("Parameter 'config_file_path' must not be empty.");
  }
  if (!pointcloud_callback_) {
    throw std::runtime_error("Point cloud callback must not be empty.");
  }

  pointcloud_msg_ = create_pointcloud_msg();
}

LivoxLidarDriverImplementation::~LivoxLidarDriverImplementation()
{
  stop();
}

void LivoxLidarDriverImplementation::start()
{
  std::lock_guard<std::mutex> lock(sdk_lifecycle_mutex);
  if (sdk_initialized_) {
    return;
  }

  if (sdk_owner != nullptr) {
    throw std::runtime_error("Only one Livox driver instance is supported per process.");
  }

  LivoxLidarSdkVer sdk_ver{};
  GetLivoxLidarSdkVer(&sdk_ver);
  sdk_version_ = std::to_string(sdk_ver.major) + "." + std::to_string(sdk_ver.minor) + "." +
    std::to_string(sdk_ver.patch);

  // Disable SDK console logger to keep ROS logs clean.
  DisableLivoxSdkConsoleLogger();

  self_handle_.store(0);
  extra_device_reported_.store(false);
  pointcloud_msg_ = create_pointcloud_msg();
  last_publish_time_ = std::chrono::system_clock::time_point{};
  // v1.3.1 starts discovery during Init and does not replay missed notifications.
  SetLivoxLidarInfoChangeCallback(&LivoxLidarDriverImplementation::GlobalInfoChangeCallback, this);
  SetLivoxLidarPointCloudCallBack(&LivoxLidarDriverImplementation::GlobalPointCloudCallback, this);
  SetLivoxLidarImuDataCallback(&LivoxLidarDriverImplementation::GlobalImuCallback, this);

  if (!LivoxLidarSdkInit(config_.config_file_path.c_str())) {
    SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
    SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
    SetLivoxLidarImuDataCallback(nullptr, nullptr);
    self_handle_.store(0);
    throw std::runtime_error(
            "Failed to initialize Livox lidar SDK with config file: " +
            config_.config_file_path);
  }

  sdk_initialized_ = true;
  sdk_owner = this;
}

void LivoxLidarDriverImplementation::stop()
{
  std::lock_guard<std::mutex> lock(sdk_lifecycle_mutex);
  if (!sdk_initialized_) {
    return;
  }

  // Uninitialize SDK and reset internal state.
  LivoxLidarSdkUninit();
  sdk_initialized_ = false;
  sdk_owner = nullptr;
  self_handle_.store(0);
  extra_device_reported_.store(false);
  pointcloud_msg_ = create_pointcloud_msg();
  last_publish_time_ = std::chrono::system_clock::time_point{};
}

bool LivoxLidarDriverImplementation::is_running() const noexcept
{
  return sdk_initialized_;
}

const std::string & LivoxLidarDriverImplementation::sdk_version() const noexcept
{
  return sdk_version_;
}

void LivoxLidarDriverImplementation::GlobalInfoChangeCallback(
  uint32_t handle, const LivoxLidarInfo * info, void * client_data)
{
  auto * impl = static_cast<LivoxLidarDriverImplementation *>(client_data);
  if (impl == nullptr || info == nullptr || handle == 0) {
    return;
  }

  uint32_t expected = 0;
  if (impl->self_handle_.compare_exchange_strong(expected, handle)) {
    RCLCPP_INFO(
      rclcpp::get_logger("fast_livox_ros_driver"),
      "Bound lidar handle=%u ip='%.*s' sn='%.*s'",
      handle, static_cast<int>(sizeof(info->lidar_ip)), info->lidar_ip,
      static_cast<int>(sizeof(info->sn)), info->sn);
  } else if (expected != handle && !impl->extra_device_reported_.exchange(true)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("fast_livox_ros_driver"),
      "Ignoring additional lidar handle=%u. Single-node multi-lidar is unsupported. "
      "Configure exactly one explicit lidar IP in the SDK JSON.", handle);
  }
}

void LivoxLidarDriverImplementation::GlobalPointCloudCallback(
  uint32_t handle,
  uint8_t dev_type,
  LivoxLidarEthernetPacket * data,
  void * client_data)
{
  auto * impl = static_cast<LivoxLidarDriverImplementation *>(client_data);
  if (impl != nullptr) {
    impl->handle_point_cloud_packet(handle, dev_type, data);
  }
}

void LivoxLidarDriverImplementation::GlobalImuCallback(
  uint32_t handle,
  uint8_t dev_type,
  LivoxLidarEthernetPacket * data,
  void * client_data)
{
  auto * impl = static_cast<LivoxLidarDriverImplementation *>(client_data);
  if (impl != nullptr) {
    impl->handle_imu_packet(handle, dev_type, data);
  }
}

void LivoxLidarDriverImplementation::handle_point_cloud_packet(
  uint32_t handle,
  uint8_t dev_type,
  LivoxLidarEthernetPacket * data)
{
  (void)dev_type;
  // Ignore packets from other devices and invalid packet pointers.
  if (data == nullptr || data->data == nullptr || handle == 0 || self_handle_.load() != handle) {
    return;
  }

  const auto packet_byte_size = get_eth_packet_byte_size(data);
  if (packet_byte_size == 0U) {
    return;
  }

  if (data->dot_num == 0U) {
    return;
  }

  // Convert SDK timing to per-point timing.
  // Livox uses 100ns units. We compute a nanosecond interval per point.
  const auto point_time_interval_ns = static_cast<uint32_t>(data->time_interval) * 100U /
    static_cast<uint32_t>(data->dot_num);
  const auto pkt_timestamp = get_eth_packet_timestamp(data->time_interval);

  if (pointcloud_msg_->width == 0U) {
    pointcloud_msg_->header.stamp = rclcpp::Time(pkt_timestamp);
  }

  append_pointcloud_from_packet(
    data->data,
    data->data_type,
    data->dot_num,
    pkt_timestamp,
    point_time_interval_ns,
    pointcloud_msg_);

  publish_pointcloud_if_ready();
}

void LivoxLidarDriverImplementation::handle_imu_packet(
  uint32_t handle,
  uint8_t dev_type,
  LivoxLidarEthernetPacket * data)
{
  (void)dev_type;
  // Ignore packets from other devices and invalid packet pointers.
  if (data == nullptr || data->data == nullptr || handle == 0 || self_handle_.load() != handle) {
    return;
  }

  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  msg->header.stamp = rclcpp::Time(get_eth_packet_timestamp(0U));
  fill_imu_message(data->data, msg);

  if (imu_callback_) {
    imu_callback_(msg);
  }
}

void LivoxLidarDriverImplementation::publish_pointcloud_if_ready()
{
  const auto publish_interval_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(kPointCloudPublishInterval);
  const auto current_time = std::chrono::system_clock::now();

  if (publish_interval_ns.count() > 0 &&
    current_time - last_publish_time_ < publish_interval_ns)
  {
    return;
  }

  // Nothing to publish yet.
  if (pointcloud_msg_ == nullptr || pointcloud_msg_->width == 0U) {
    return;
  }

  pointcloud_msg_->row_step = pointcloud_msg_->width * pointcloud_msg_->point_step;
  pointcloud_callback_(pointcloud_msg_);
  last_publish_time_ = current_time;

  pointcloud_msg_ = create_pointcloud_msg();
}

std::size_t LivoxLidarDriverImplementation::get_eth_packet_byte_size(
  const LivoxLidarEthernetPacket * data) const
{
  if (data == nullptr || data->data == nullptr) {
    return 0U;
  }

  switch (data->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      return sizeof(LivoxLidarCartesianHighRawPoint) * data->dot_num;
    case kLivoxLidarCartesianCoordinateLowData:
      return sizeof(LivoxLidarCartesianLowRawPoint) * data->dot_num;
    case kLivoxLidarSphericalCoordinateData:
      return sizeof(LivoxLidarSpherPoint) * data->dot_num;
    default:
      return 0U;
  }
}

uint64_t LivoxLidarDriverImplementation::get_eth_packet_timestamp(
  uint32_t packet_time_interval_100ns) const
{
  const auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
  return now_ns - static_cast<uint64_t>(packet_time_interval_100ns) *
         kHundredNanosecondsToNanoseconds;
}

sensor_msgs::msg::PointCloud2::SharedPtr LivoxLidarDriverImplementation::create_pointcloud_msg()
const
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->height = kPointCloudHeight;
  msg->width = 0;
  msg->is_bigendian = false;
  msg->point_step = sizeof(CloudPointXYZIT);
  msg->row_step = 0;
  msg->is_dense = true;

  msg->fields.resize(kPointCloudFieldCount);
  msg->fields[kXFieldIndex].name = "x";
  msg->fields[kXFieldIndex].offset = offsetof(CloudPointXYZIT, x);
  msg->fields[kXFieldIndex].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[kXFieldIndex].count = kSingleFieldElementCount;

  msg->fields[kYFieldIndex].name = "y";
  msg->fields[kYFieldIndex].offset = offsetof(CloudPointXYZIT, y);
  msg->fields[kYFieldIndex].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[kYFieldIndex].count = kSingleFieldElementCount;

  msg->fields[kZFieldIndex].name = "z";
  msg->fields[kZFieldIndex].offset = offsetof(CloudPointXYZIT, z);
  msg->fields[kZFieldIndex].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[kZFieldIndex].count = kSingleFieldElementCount;

  msg->fields[kIntensityFieldIndex].name = "intensity";
  msg->fields[kIntensityFieldIndex].offset = offsetof(CloudPointXYZIT, intensity);
  msg->fields[kIntensityFieldIndex].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[kIntensityFieldIndex].count = kSingleFieldElementCount;

  msg->fields[kTagFieldIndex].name = "tag";
  msg->fields[kTagFieldIndex].offset = offsetof(CloudPointXYZIT, tag);
  msg->fields[kTagFieldIndex].datatype = sensor_msgs::msg::PointField::UINT8;
  msg->fields[kTagFieldIndex].count = kSingleFieldElementCount;

  msg->fields[kTimestampFieldIndex].name = "timestamp";
  msg->fields[kTimestampFieldIndex].offset = offsetof(CloudPointXYZIT, timestamp);
  msg->fields[kTimestampFieldIndex].datatype = sensor_msgs::msg::PointField::FLOAT64;
  msg->fields[kTimestampFieldIndex].count = kSingleFieldElementCount;

  msg->data.reserve(kMaxPointCloudCapacity * sizeof(CloudPointXYZIT));
  return msg;
}

void LivoxLidarDriverImplementation::append_pointcloud_from_packet(
  const unsigned char * data,
  int data_type,
  uint32_t point_count,
  uint64_t pkt_timestamp,
  uint32_t point_time_interval_ns,
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg) const
{
  if (msg == nullptr) {
    throw std::runtime_error("PointCloud2 message must not be null.");
  }

  // The driver currently supports only Cartesian high precision packets.
  // More formats can be added by branching here.
  if (data_type != kLivoxLidarCartesianCoordinateHighData) {
    throw std::runtime_error("Unsupported Livox data type: " + std::to_string(data_type));
  }

  // Interpret the raw byte buffer as an array of SDK point structs.
  auto * raw_points = reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(data);
  const auto current_size = msg->data.size() / sizeof(CloudPointXYZIT);
  const auto new_size = current_size + point_count;
  if (new_size > kMaxPointCloudCapacity) {
    throw std::runtime_error(
            "Point cloud size exceeds internal point cloud capacity: " + std::to_string(new_size));
  }

  msg->data.resize(new_size * sizeof(CloudPointXYZIT));
  msg->width = static_cast<uint32_t>(new_size);

  // Write into the new tail region of msg->data.
  auto * cloud_points = reinterpret_cast<CloudPointXYZIT *>(
    msg->data.data() + current_size * sizeof(CloudPointXYZIT));
  for (uint32_t i = 0; i < point_count; ++i) {
    // Convert mm -> m and build per-point timestamp.
    cloud_points[i].x = raw_points[i].x * kMillimetersToMeters;
    cloud_points[i].y = raw_points[i].y * kMillimetersToMeters;
    cloud_points[i].z = raw_points[i].z * kMillimetersToMeters;
    cloud_points[i].intensity = static_cast<float>(raw_points[i].reflectivity);
    cloud_points[i].tag = raw_points[i].tag;
    cloud_points[i].timestamp = static_cast<double>(pkt_timestamp +
      static_cast<uint64_t>(i) * point_time_interval_ns);
  }
}

void LivoxLidarDriverImplementation::fill_imu_message(
  const unsigned char * data,
  const sensor_msgs::msg::Imu::SharedPtr & msg) const
{
  if (msg == nullptr) {
    throw std::runtime_error("Imu message must not be null.");
  }

  // Interpret raw bytes as a packed IMU sample.
  auto * imu = reinterpret_cast<const RawImuPoint *>(data);
  msg->angular_velocity.x = imu->gyro_x;
  msg->angular_velocity.y = imu->gyro_y;
  msg->angular_velocity.z = imu->gyro_z;
  // Livox accelerometer is proportional to g; convert to m/s^2.
  msg->linear_acceleration.x = imu->acc_x * kStandardGravity;
  msg->linear_acceleration.y = imu->acc_y * kStandardGravity;
  msg->linear_acceleration.z = imu->acc_z * kStandardGravity;
}

}  // namespace fast_livox_ros_driver
