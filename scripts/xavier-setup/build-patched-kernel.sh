#!/bin/bash
set -e

BUILD_SCRIPTS_DIR="/home/nvidia/buildLibrealsense2Xavier/"
if [ ! -d "$BUILD_SCRIPTS_DIR" ]; then
    git clone https://github.com/jetsonhacks/buildLibrealsense2Xavier.git "$BUILD_SCRIPTS_DIR"
fi

# Downgrade bzip2 in order to be able to unzip kernel source tarball
sudo apt install bzip2=1.0.6-8.1 libbz2-1.0=1.0.6-8.1

pushd "$BUILD_SCRIPTS_DIR"

./buildPatchedKernel.sh

popd

echo "Build complete. Copy the built file to a host machine and flash it."
