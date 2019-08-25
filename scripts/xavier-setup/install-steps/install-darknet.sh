#!/bin/bash
set -e

BUILD_SCRIPTS_DIR="/home/nvidia/aruw-vision-platform-2019/darknet-aruw/"

pushd "$BUILD_SCRIPTS_DIR"
make -j8

popd

echo 'export PYTHONPATH='"$BUILD_SCRIPTS_DIR"'python:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc

echo "Install complete."
