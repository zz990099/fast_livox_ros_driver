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

#include <memory>
#include <stdexcept>
#include <string>

#include "fast_livox_ros_driver/livox_lidar_driver_impl.hpp"

namespace fast_livox_ros_driver
{

TEST(test_driver_config_validation, throws_on_empty_config_file_path)
{
  LivoxDriverConfig cfg;
  cfg.config_file_path = "";

  auto pc_cb = [](sensor_msgs::msg::PointCloud2::SharedPtr) {};
  auto imu_cb = [](sensor_msgs::msg::Imu::SharedPtr) {};

  EXPECT_THROW(
    (void)std::make_unique<LivoxLidarDriverImplementation>(cfg, pc_cb, imu_cb),
    std::runtime_error);
}

TEST(test_driver_config_validation, throws_on_empty_pointcloud_callback)
{
  LivoxDriverConfig cfg;
  cfg.config_file_path = "/tmp/livox.json";

  LivoxLidarDriverImplementation::PointCloudCallback pc_cb;
  auto imu_cb = [](sensor_msgs::msg::Imu::SharedPtr) {};

  EXPECT_THROW(
    (void)std::make_unique<LivoxLidarDriverImplementation>(cfg, pc_cb, imu_cb),
    std::runtime_error);
}

}  // namespace fast_livox_ros_driver
