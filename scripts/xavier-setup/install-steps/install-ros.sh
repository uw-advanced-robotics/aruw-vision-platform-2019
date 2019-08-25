#!/bin/bash
set -e

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

BUILD_SCRIPTS_DIR="/home/nvidia/installROSXavier/"
if [ ! -d "$BUILD_SCRIPTS_DIR" ]; then
    git clone https://github.com/jetsonhacks/installROSXavier "$BUILD_SCRIPTS_DIR"
fi

pushd "$BUILD_SCRIPTS_DIR"
./installROS.sh
popd

sudo apt install ros-melodic-geometry2 ros-melodic-tf-conversions -y



