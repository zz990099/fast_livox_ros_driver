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
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_FLAGS="-include cstdint" && make -j
sudo make install
```

The SDK configure flags support CMake 4 and supply the `<cstdint>` header
omitted by SDK v1.3.1 when building with newer compilers.

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

Each node supports exactly one lidar. Single-node multi-lidar operation is
unsupported. Run each lidar in a separate process with its own SDK JSON,
topics, and frame_id. Multiple driver components in one process are rejected
because the SDK uses process-global callbacks.

Set exactly one explicit lidar IP in the SDK JSON using
`MID360.host_net_info[0].lidar_ip`, as shown in
[`config/MID360_config.json`](config/MID360_config.json). The array must contain
one IP. Use `host_ip` for the receiving computer and remove generic discovery
entries that have no explicit lidar IP. For other devices, use the model key
recognized by SDK v1.3.1.

The driver passes the JSON path directly to the SDK without parsing the file.
It binds the first valid SDK device-info handle or the first supported
pointcloud/IMU data callback handle, then accepts only that handle.
Binding from SDK-selected data also supports receiving-only configurations
(`master_sdk: false`), which do not emit device-info notifications in v1.3.1.
Empty pointcloud packets, unsupported data types, and zero handles cannot bind. Additional devices
are ignored and reported once per start. The driver cannot validate the
number of configured devices, so a JSON containing multiple IPs is unsupported
even when only one device is currently online. Stop/start clears the binding
and accumulated pointcloud data.

When migrating, remove the ROS `lidar_ip` parameter and move the device IP from
the old top-level `lidar_configs[].ip` into the SDK-native
`host_net_info[].lidar_ip` array. SDK v1.3.1 does not use the old top-level
field to select a device. Update both files together.

The node exposes the following ROS parameters.

- `frame_id` (string): TF frame used for published messages.
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
ros2 launch fast_livox_ros_driver fast_livox_driver.launch.py
```

For multiple processes, give each a distinct SDK config path through
`livox_config_file:=/path/to/lidar.json`. Configure distinct topics and frame
IDs in each ROS parameter file using `params_file:=/path/to/params.yaml`.
The default topics are absolute, so a namespace alone does not separate them.
Plan host UDP ports for the processes to avoid socket conflicts.

## Testing

Unit and integration tests (if present) can be run with colcon:

```bash
colcon build --packages-select fast_livox_ros_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fast_livox_ros_driver
```

## License

This project is released under the Apache-2.0 license. See the [`LICENSE`](./LICENSE)
file for full terms.
