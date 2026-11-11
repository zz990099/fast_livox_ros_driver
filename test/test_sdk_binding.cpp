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

#include <gtest/gtest.h>
#include <livox_lidar_api.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "fast_livox_ros_driver/livox_lidar_driver_impl.hpp"

namespace
{
LivoxLidarInfoChangeCallback info_callback = nullptr;
LivoxLidarPointCloudCallBack cloud_callback = nullptr;
LivoxLidarImuDataCallback imu_callback = nullptr;
void * info_context = nullptr;
void * cloud_context = nullptr;
void * imu_context = nullptr;
bool init_success = true;
uint32_t init_handle = 0;
int uninit_count = 0;

void notify(uint32_t handle)
{
  LivoxLidarInfo info{};
  std::strcpy(info.lidar_ip, "192.168.1.12");
  std::strcpy(info.sn, "test-device");
  ASSERT_NE(info_callback, nullptr);
  info_callback(handle, &info, info_context);
}

std::vector<uint8_t> point_packet()
{
  std::vector<uint8_t> bytes(
    sizeof(LivoxLidarEthernetPacket) + sizeof(LivoxLidarCartesianHighRawPoint));
  auto * packet = reinterpret_cast<LivoxLidarEthernetPacket *>(bytes.data());
  packet->data_type = kLivoxLidarCartesianCoordinateHighData;
  packet->dot_num = 1;
  LivoxLidarCartesianHighRawPoint point{};
  point.x = 1000;
  std::memcpy(packet->data, &point, sizeof(point));
  return bytes;
}

void send_cloud(uint32_t handle)
{
  auto bytes = point_packet();
  ASSERT_NE(cloud_callback, nullptr);
  cloud_callback(
    handle, 0, reinterpret_cast<LivoxLidarEthernetPacket *>(bytes.data()), cloud_context);
}

void send_imu(uint32_t handle)
{
  std::vector<uint8_t> bytes(sizeof(LivoxLidarEthernetPacket) + 6 * sizeof(float));
  ASSERT_NE(imu_callback, nullptr);
  imu_callback(handle, 0, reinterpret_cast<LivoxLidarEthernetPacket *>(bytes.data()), imu_context);
}
}  // namespace

// Link the production implementation against these SDK entry points. Discovery
// inside Init catches late registration without using a network or a device.
void GetLivoxLidarSdkVer(LivoxLidarSdkVer * version)
{
  *version = LivoxLidarSdkVer{1, 3, 1};
}
void DisableLivoxSdkConsoleLogger() {}
void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void * context)
{
  info_callback = cb;
  info_context = context;
}
void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void * context)
{
  cloud_callback = cb;
  cloud_context = context;
}
void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void * context)
{
  imu_callback = cb;
  imu_context = context;
}
bool LivoxLidarSdkInit(const char *, const char *, const LivoxLidarLoggerCfgInfo *)
{
  EXPECT_NE(info_callback, nullptr);
  EXPECT_NE(cloud_callback, nullptr);
  EXPECT_NE(imu_callback, nullptr);
  if (init_handle != 0) {
    notify(init_handle);
  }
  return init_success;
}
void LivoxLidarSdkUninit()
{
  ++uninit_count;
  // Exercise a final callback while the driver context must still be alive.
  send_imu(init_handle);
  SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
  SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
  SetLivoxLidarImuDataCallback(nullptr, nullptr);
}

class SdkBindingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    init_success = true;
    init_handle = 0;
    uninit_count = 0;
  }

  std::unique_ptr<fast_livox_ros_driver::LivoxLidarDriverImplementation> driver()
  {
    fast_livox_ros_driver::LivoxDriverConfig config;
    config.config_file_path = "/unused/sdk.json";
    return std::make_unique<fast_livox_ros_driver::LivoxLidarDriverImplementation>(
      config,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {clouds.push_back(msg);},
      [this](sensor_msgs::msg::Imu::SharedPtr) {++imus;});
  }

  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> clouds;
  int imus = 0;
};

TEST_F(SdkBindingTest, waits_for_info_and_filters_both_streams)
{
  auto impl = driver();
  impl->start();
  send_cloud(11);
  send_imu(11);
  send_cloud(0);
  send_imu(0);
  EXPECT_TRUE(clouds.empty());
  EXPECT_EQ(imus, 0);

  notify(0);
  notify(11);
  send_cloud(22);
  send_imu(22);
  send_cloud(11);
  send_imu(11);
  ASSERT_EQ(clouds.size(), 1U);
  EXPECT_EQ(clouds.front()->width, 1U);
  EXPECT_EQ(imus, 1);
}

TEST_F(SdkBindingTest, binds_during_init_and_ignores_additional_devices)
{
  init_handle = 11;
  auto impl = driver();
  impl->start();
  impl->start();
  notify(11);
  notify(22);
  send_cloud(22);
  send_imu(22);
  send_cloud(11);
  send_imu(11);
  ASSERT_EQ(clouds.size(), 1U);
  EXPECT_EQ(clouds.front()->width, 1U);
  EXPECT_EQ(imus, 1);
}

TEST_F(SdkBindingTest, stop_clears_binding_and_buffer_before_restart)
{
  init_handle = 11;
  auto impl = driver();
  impl->start();
  send_cloud(11);  // First packet publishes immediately.
  send_cloud(11);  // Second packet remains in the aggregation buffer.
  impl->stop();
  impl->stop();
  EXPECT_FALSE(impl->is_running());
  EXPECT_EQ(uninit_count, 1);

  init_handle = 0;
  impl->start();
  send_cloud(11);
  send_imu(11);
  EXPECT_EQ(clouds.size(), 1U);
  const int previous_imus = imus;
  notify(22);
  send_cloud(22);
  send_imu(22);
  ASSERT_EQ(clouds.size(), 2U);
  EXPECT_EQ(clouds.back()->width, 1U);
  EXPECT_EQ(imus, previous_imus + 1);
}

TEST_F(SdkBindingTest, failed_init_removes_callbacks_and_allows_retry)
{
  init_success = false;
  init_handle = 11;
  auto impl = driver();
  EXPECT_THROW(impl->start(), std::runtime_error);
  EXPECT_FALSE(impl->is_running());
  EXPECT_EQ(info_callback, nullptr);
  EXPECT_EQ(cloud_callback, nullptr);
  EXPECT_EQ(imu_callback, nullptr);
  EXPECT_EQ(info_context, nullptr);
  EXPECT_EQ(cloud_context, nullptr);
  EXPECT_EQ(imu_context, nullptr);
  init_success = true;
  init_handle = 0;
  impl->start();
  send_cloud(11);
  send_imu(11);
  EXPECT_TRUE(clouds.empty());
  EXPECT_EQ(imus, 0);
  notify(22);
  send_imu(22);
  EXPECT_EQ(imus, 1);
}

TEST_F(SdkBindingTest, second_instance_cannot_replace_global_callbacks)
{
  init_handle = 11;
  auto first = driver();
  first->start();
  {
    auto second = driver();
    EXPECT_THROW(second->start(), std::runtime_error);
  }
  EXPECT_EQ(uninit_count, 0);
  send_imu(11);
  EXPECT_EQ(imus, 1);
}
