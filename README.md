# aruw-vision-platform-2019

## Introduction

This repository includes all of the vision-related detection, tracking and aiming code from ARUW's 2019 season. This system is responsible for identifying target plates in video footage, calculating a 3D position in the world for that plate, and computing the angle the turret must point to hit the target.

It uses a Machine Learning (ML) model to recognize targets, then uses the Intel RealSense deepth feed to compute the 3D point where the plate exists in space. It fuses that point with calculated odometry to negate our own robot movement, filters the location data to minimize noise, and then computes the aim angles necessary to account for the target's motion while the projectile flies through the air. The goal with these additional calculations, as opposed to the traditional style of pointing directly at the target, is to make our vision system effective at long range.

## Capabilities and Results (Software effects display)

Our system is unique among RoboMaster teams' vision systems in three primary ways:
- We use a machine learning approach to plate detection rather than classical CV. This enables detection in much more diverse environments and positions with minimal manual intervention. There is no brightness of threshold tuning, and extremely angled plates can still be identified.
- We use a depth camera and robot odometry to track targets in world-relative 3D space. This allows us to do more intelligent correction and prediction than would be possible with a naive camera-frame-relative alignment approach.
- We do ballistics calculation and correction to account for gravity, target motion, chassis motion, and bullet velocity. This becomes more significant at longer range.

![Sentinel Test](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/blob/master/.github/sentinel_practice-opt.gif?raw=true)
![Real Match Result 1](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/blob/master/.github/ohio23-opt.gif?raw=true)
![Real Match Result 2](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/blob/master/.github/ohio48-opt.gif?raw=true)

## Dependencies and Hardware/Software environment

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

See below for installation instructions -- most of it is done by the provided scripts.

## Compilation and Installation

Follow the instructions in `scripts/xavier-setup/README.md` to set up the depdencies for an Xavier.

To run the whole production system on the Xavier, run `roslaunch aruw_common prod.launch`. To only do detection without any serial communication, you can replace `prod.launch` with `dev.launch`.

To enable auto-startup for production use in a real match, enter the `scripts` folder and run `./configure-service.sh -e`. This will configure the app as a service that runs on boot. To disable the service, run `./configure-service.sh`.

Our setup scripts will configure Bash aliases for interacting with the service:
- `vision-status`: displays the running/stopped status of the service and the most recent log output.
- `vision-start`: start the service.
- `vision-stop`: stop the service.
- `vision-log`: `less` the log output from the service.

## Structure and Organization

See our Wiki:  [Software Architecture: File Structure and Organization](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Software-Architecture:-File-Structure-and-Organization)

## Software and Hardware Block Diagrams and Data Flow

See our Wiki: [Block Diagrams and Data Flow](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Block-Diagrams-and-Data-Flow). Additionally, see these accompanying pages:

- [Custom ROS Messages](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Custom-ROS-Messages): Index of our custom ROS message classes.
- [Serial Data Protocol](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Serial-Data-Protocol): The byte protocol used in serial communication.

## Principle Introduction and Theoretical Support Analysis

In addition to the architecture information in past sections, see our Wiki for each of the following:
- [Armor Plate Detector](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Principle-Introduction-and-Theoretical-Support-Analysis:-Armor-Plate-Detector)
- [Ballistics](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Principle-Introduction-and-Theoretical-Support-Analysis:-Ballistics)
- [Linear Kalman Filters](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Principle-Introduction-and-Theoretical-Support-Analysis:-Linear-Kalman-Filters)
- [Target Tracking and Target Selection](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Principle-Introduction-and-Theoretical-Support-Analysis:-Target-Tracking-and-Target-Selection)

## Software Architecture and Hierachy diagram

See our Wiki for relevant pages:

- [Software Architecture: File Structure and Organization](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Software-Architecture:-File-Structure-and-Organization)
- [Block Diagrams and Data Flow](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Block-Diagrams-and-Data-Flow)
- [Custom ROS Messages](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Custom-ROS-Messages): Index of our custom ROS message classes.
- [Serial Data Protocol](https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki/Serial-Data-Protocol): The byte protocol used in serial communication.

## Roadmap

We have a strong foundation which supports advanced use-cases for vision in the RoboMaster competition. However, there are many areas we can improve on to increase accuracy, responsiveness and consistency:

- Make the system more robust to RealSense instability. This requires investigating interop issues at the operating system and library level.
- Decrease round-trip outer-loop time:
  - Optimize execution of neural network detector (both the wrapper code and actual inference time)
  - Investigate moving ballistics and odometry to main controller, and send only 3D position and velocity rather than turret aim.
- Improve precision of frame and odometry timestamps (eliminate fudge factors)
- Unit tests and simulations for all major components

There are also plans and directions to improve individual modules as well located in their respective wiki pages: https://github.com/uw-advanced-robotics/aruw-vision-platform-2019/wiki
