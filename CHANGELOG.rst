Changelog
=========

1.2.0 (2026-11-12)
------------------

Breaking changes and migration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove the ROS parameter and C++ configuration field ``lidar_ip``.
  The driver now obtains the device handle from SDK callbacks.
* Move the target IP from the old top-level ``lidar_configs[].ip``
  into the SDK-native ``MID360.host_net_info[0].lidar_ip`` array.
  Use exactly one IP and remove generic discovery entries.
* Each node/process supports one lidar. Multiple driver instances in a
  process are rejected to protect the SDK's global callbacks.
  Multiple lidars require separate processes, configs, topics, and frame IDs.

Changes
^^^^^^^

* Bind a shared handle for pointcloud and IMU filtering through device-info
  or supported data callbacks, including receiving-only ``master_sdk: false``.
* Register callbacks before SDK initialization, clear callbacks on failed
  initialization, and reset binding and pointcloud accumulation on restart.
* Add hardware-free regression tests for binding, filtering, receiving-only
  streams, initialization failure, restart, and process-global ownership.
* Correct the packaged launch action and executable name, document runnable
  commands with a required SDK config path, and add installed-package CI checks.
* Declare the missing ``rclcpp_components`` dependency and correct the
  exported include path for downstream C++ consumers.
* Update the ROS package version to 1.2.0.

Dependencies and validation
^^^^^^^^^^^^^^^^^^^^^^^^^^

* Continue using official Livox-SDK2 v1.3.1 and the ROS 2 LTS CI matrix
  of Humble, Jazzy, and Lyrical.
* Hardware verification is outside this release preparation's validation scope.

1.0.0 (2026-03-24)
------------------

Initial public release of ``fast_livox_ros_driver``.

Features
^^^^^^^^

* High-performance ROS 2 driver for Livox LiDAR sensors.
* Publishes pointclouds as ``sensor_msgs/msg/PointCloud2``.
* Publishes IMU data as ``sensor_msgs/msg/Imu`` when available.
* Supports configuration via ROS parameters and packaged config/launch files.
* Component/node library plus a standalone executable entrypoint.
* Apache-2.0 licensed.
