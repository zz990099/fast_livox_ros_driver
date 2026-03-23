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

#include <string>

#include "fast_livox_ros_driver/livox_lidar_driver_impl.hpp"

namespace fast_livox_ros_driver
{

class LivoxLidarDriverImplementationTestable : public LivoxLidarDriverImplementation
{
public:
  using LivoxLidarDriverImplementation::LivoxLidarDriverImplementation;
  using LivoxLidarDriverImplementation::get_handle_from_ip;
};

TEST(test_ipv4_handle, throws_on_invalid_ip_format)
{
  LivoxDriverConfig cfg;
  cfg.lidar_ip = "192.168.1.10";
  cfg.config_file_path = "/tmp/livox.json";

  auto pc_cb = [](sensor_msgs::msg::PointCloud2::SharedPtr) {};
  auto imu_cb = [](sensor_msgs::msg::Imu::SharedPtr) {};

  LivoxLidarDriverImplementationTestable impl(cfg, pc_cb, imu_cb);

  EXPECT_THROW((void)impl.get_handle_from_ip("not_an_ip"), std::runtime_error);
  EXPECT_THROW((void)impl.get_handle_from_ip("192.168.1"), std::runtime_error);
  EXPECT_THROW((void)impl.get_handle_from_ip("256.1.2.3"), std::runtime_error);
}

TEST(test_ipv4_handle, deterministic_handle_for_valid_ip)
{
  LivoxDriverConfig cfg;
  cfg.lidar_ip = "192.168.1.10";
  cfg.config_file_path = "/tmp/livox.json";

  auto pc_cb = [](sensor_msgs::msg::PointCloud2::SharedPtr) {};
  auto imu_cb = [](sensor_msgs::msg::Imu::SharedPtr) {};

  LivoxLidarDriverImplementationTestable impl(cfg, pc_cb, imu_cb);

  // Implementation uses little-endian packing per octet order in the string:
  // handle = o1 << 0 | o2 << 8 | o3 << 16 | o4 << 24
  const uint32_t expected =
    (192u << 0) |
    (168u << 8) |
    (1u << 16) |
    (10u << 24);

  EXPECT_EQ(impl.get_handle_from_ip("192.168.1.10"), expected);
}

}  // namespace fast_livox_ros_driver
