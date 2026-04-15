---
name: Hardware Testing Report
about: Report test results for fast_livox_ros_driver on a new hardware platform
title: "[Hardware Test] <platform name>"
labels: help wanted
assignees: ''

---

## Platform Details

- **CPU model**: <!-- e.g. Raspberry Pi CM4 (BCM2711), NVIDIA Jetson Orin, ... -->
- **Architecture**: <!-- e.g. aarch64, armv7l, riscv64, ... -->
- **OS and version**: <!-- e.g. Ubuntu 22.04, Debian Bookworm, ... -->
- **ROS 2 distro**: <!-- e.g. Humble, Iron, Jazzy, Rolling -->

## Build Result

<!-- Did `colcon build` succeed? Any warnings or errors? -->

- [ ] Build succeeded
- [ ] Build failed (describe below)

<details>
<summary>Build log (if relevant)</summary>

```
<paste output here>
```

</details>

## Runtime Result

<!-- Briefly describe whether the driver started, published pointclouds/IMU data, and any observed issues. -->

- [ ] Driver started without errors
- [ ] Pointcloud data published correctly
- [ ] IMU data published correctly (if applicable)
- [ ] Encountered issues (describe below)

## CPU Usage

| Driver | CPU usage (%) |
|---|---|
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | <!-- e.g. 30% --> |
| fast_livox_ros_driver | <!-- e.g. 12% --> |

<!-- Measurement method: e.g. `top`, `htop`, `pidstat`, ROS 2 `ros2 run rqt_top rqt_top` -->

## Additional Notes

<!-- Any other observations, caveats, or suggestions. -->
