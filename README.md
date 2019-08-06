# aruw-vision-platform-2019

This repository includes all of the vision-related detection, tracking and aiming code from ARUW's 2019 season. This system is responsible for identifying target plates in video footage, calculating a 3D position in the world for that plate, and computing the angle the turret must point to hit the target.

## Capabilities and Results

Our system is largely unique among RoboMaster teams' vision systems in three primary ways:
- We use a machine learning approach to plate detection rather than classical CV. This enables detection in much more diverse environments and positions with minimal manual intervention.
- We use a depth camera and robot odometry to track targets in world-relative 3D space. This allows us to do more intelligent correction and prediction than would be possible with a naive camera-frame-relative alignment approach.
- We do ballistics calculation and correction to account for gravity, target motion, chassis motion, and bullet velocity. This becomes more significant at longer range.

**TODO: Images and GIFs**

## Dependencies

This is the hardware and software we use on our production robots. It is certainly possible to swap out components with minimal effort if desired (e.g., disable GPU support if there is no GPU available, use alternate camera SDK if no RealSense is available).

Hardware:
- Linux environment (tested on Ubuntu 18.04, other platforms may also work)
- Intel RealSense D435
- NVIDIA Jetson Xavier DevKit
- Serial (UART) connection to the main controller

Software:
- ROS Melodic
- librealsense 2.24+
- `darknet-aruw` (our custom fork with minor fixes)

## Setup

Follow the instructions in `scripts/xavier-setup/README.md`.

To run the whole production system, run `roslaunch aruw_common prod.launch`. To only do detection without any serial communication, you can replace `prod.launch` with `dev.launch`.
