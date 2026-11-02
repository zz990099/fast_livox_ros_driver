# fast_livox_ros_driver

A lightweight, high-performance ROS 2 driver for Livox LiDAR sensors.

This package exposes Livox pointcloud and IMU data as standard
`sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/Imu` messages. It is
designed for efficiency and low-latency use in production and research.

## CPU usage (before/after)

The following numbers are measured in our internal benchmarks and are provided
as a reference for expected CPU usage improvements.

| Platform | CPU usage ([livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)) | CPU usage (fast_livox_ros_driver) |
|---|---:|---:|
| RK3588 | 35% | ***15%*** |
| Intel i7-11800H | 14% | ***3%*** |

## Features

- Publishes pointclouds (`sensor_msgs/msg/PointCloud2`).
- Publishes IMU messages (`sensor_msgs/msg/Imu`) when available.
- Configurable frame, topics, QoS and SDK configuration file.

## Supported ROS 2 Distros and Platforms

This package targets supported ROS 2 LTS distributions: Humble, Jazzy and
Lyrical. CI builds and runs the test suite on Linux using the official ROS
container images for the following Ubuntu versions. Other platforms
(Windows/macOS) are not tested.

[![ROS 2 Build & Test](https://github.com/zz990099/fast_livox_ros_driver/actions/workflows/ci_ros2_build_test.yml/badge.svg?branch=master&event=push)](https://github.com/zz990099/fast_livox_ros_driver/actions/workflows/ci_ros2_build_test.yml)

The badge shows the overall workflow status across all three distributions.

| ROS 2 LTS distro | Ubuntu |
|---|---|
| Humble | 22.04 (Jammy) |
| Jazzy | 24.04 (Noble) |
| Lyrical | 26.04 (Resolute) |

## Building from source

1. Build and install the official [`Livox-SDK2 v1.3.1`](https://github.com/Livox-SDK/Livox-SDK2/releases/tag/v1.3.1), matching the version used in CI. See the [versioned SDK instructions](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.3.1/README.md).

```bash
git clone --depth 1 --branch v1.3.1 https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -j
sudo make install
```

2. Clone the repository into your workspace `src/`:

```bash
cd /path/to/colcon_ws/src
git clone https://github.com/zz990099/fast_livox_ros_driver.git fast_livox_ros_driver
```

3. Build with colcon:

```bash
colcon build --packages-select fast_livox_ros_driver
source install/setup.bash
```

## Configuration & Parameters

The node exposes several parameters to configure networking, topics and QoS.

- `frame_id` (string): TF frame used for published messages.
- `lidar_ip` (string): IP address of the Livox device.
- `config_file_path` (string): Path to a Livox SDK configuration file.
- `pointcloud_topic` (string): Topic name for pointcloud output (default: `/livox/lidar`).
- `imu_topic` (string): Topic name for IMU output (default: `/livox/imu`).
- `pointcloud_qos_depth` (int): QoS history depth for pointcloud publisher.
- `imu_qos_depth` (int): QoS history depth for imu publisher.

See the packaged `config` directory and launch files for example parameter files.

## Running

Run the node directly after sourcing your workspace:

```bash
ros2 run fast_livox_ros_driver fast_livox_ros_driver_node
```

Or use a params file:

```bash
ros2 run fast_livox_ros_driver fast_livox_ros_driver_node --ros-args --params-file /path/to/params.yaml
```

Use the packaged launch file to start the node with example params and topic remapping:

```bash
ros2 launch fast_livox_ros_driver driver.launch.py
```

To run multiple instances, remap the namespace for each instance.

## Testing

Unit and integration tests (if present) can be run with colcon:

```bash
colcon build --packages-select fast_livox_ros_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fast_livox_ros_driver
```

## License

This project is released under the Apache-2.0 license. See the [`LICENSE`](./LICENSE)
file for full terms.
