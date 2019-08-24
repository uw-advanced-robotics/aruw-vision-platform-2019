# aruw-vision-platform-2019

## Introduction

This repository includes all of the vision-related detection, tracking and aiming code from ARUW's 2019 season. This system is responsible for identifying target plates in video footage, calculating a 3D position in the world for that plate, and computing the angle the turret must point to hit the target.

## Capabilities and Results

Our system is largely unique among RoboMaster teams' vision systems in three primary ways:
- We use a machine learning approach to plate detection rather than classical CV. This enables detection in much more diverse environments and positions with minimal manual intervention.
- We use a depth camera and robot odometry to track targets in world-relative 3D space. This allows us to do more intelligent correction and prediction than would be possible with a naive camera-frame-relative alignment approach.
- We do ballistics calculation and correction to account for gravity, target motion, chassis motion, and bullet velocity. This becomes more significant at longer range.

![Sentinel Test](https://github.com/WasabiFan/aruw-vision-platform-2019/blob/master/.github/sentinel_practice-opt.gif?raw=true)
![Real Match Result 1](https://github.com/WasabiFan/aruw-vision-platform-2019/blob/master/.github/ohio23-opt.gif?raw=true)
![Real Match Result 2](https://github.com/WasabiFan/aruw-vision-platform-2019/blob/master/.github/ohio48-opt.gif?raw=true)

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
- JetPack 4.2.1 (includes OpenCV 3.3.1, CUDA 10.0.326)

## Compilation and Installation

Follow the instructions in `scripts/xavier-setup/README.md` to setup the depdencies for an xavier.

To run the whole production system on the xavier, run `roslaunch aruw_common prod.launch`. To only do detection without any serial communication, you can replace `prod.launch` with `dev.launch`.

To enable auto-startup for production use in a real match, enter the `scripts` folder and run `./configure-service.sh -e`. This will configure the app as a service that runs on boot. To disable the service, rune `./configure-service.sh`

## Structure and Organization

See our Wiki: https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Structure-and-Organization

## Principle Introduction and Theoretical Support Analysis

See our Wiki: https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki

Each module's theoretical support can be found under its own wiki page

## Software Architecture and Hierachy diagram

See our Wiki: https://github.com/WasabiFan/aruw-vision-platform-2019/wiki/Software-and-Hardware-Block-Diagrams

## Roadmap

We have a strong foundation which supports advanced use-cases for vision in the RoboMaster competition. However, there are many areas we can improve on to increase accuracy, responsiveness and consistency:

- Make the system more robust to RealSense instability. This requires investigating interop issues at the operating system and library level.
- Decrease round-trip outer-loop time:
  - Optimize execution of neural network detector (both the wrapper code and actual inference time)
  - Investigate moving ballistics and odometry to main controller, and send only 3D position and velocity rather than turret aim.
- Improve precision of frame and odometry timestamps (eliminate fudge factors)
- Unit tests and simulations for all major components

There are also plans and directions to improve individual modules as well located in their respective wikis: https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki
