#!/bin/bash
set -e

REPO_DIR="/home/nvidia/catkin_ws/src/aruw-vision-platform-2019/"
source ~/.bashrc

mkdir -p ~/catkin_ws/src

pushd ~/catkin_ws/

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

popd

echo "Install complete. Cloning repo..."

if [ ! -d "$REPO_DIR" ]; then
    git clone --recursive https://github.com/uw-advanced-robotics/aruw-vision-platform-2019.git "$REPO_DIR"
fi

pushd ~/catkin_ws
catkin_make
popd

pushd ~/catkin_ws/src/aruw-vision-platform-2019/darknet-aruw/
make -j8
popd