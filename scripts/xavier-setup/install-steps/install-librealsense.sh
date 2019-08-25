#!/bin/bash
set -e

BUILD_SCRIPTS_DIR="/home/nvidia/buildLibrealsense2Xavier/"
if [ ! -d "$BUILD_SCRIPTS_DIR" ]; then
    # our fork includes custom fixes not in upstream
    git clone https://github.com/wasabifan/buildLibrealsense2Xavier.git "$BUILD_SCRIPTS_DIR"
fi

pushd "$BUILD_SCRIPTS_DIR"
./installLibrealsense.sh

popd

echo 'export PYTHONPATH=/usr/local/lib:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc

echo "Install complete."
