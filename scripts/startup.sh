#!/bin/bash

set -e

echo
echo
echo "NEW RUN ===================="

# kill the children if our service is stopped
# https://stackoverflow.com/questions/33325301/killing-a-bash-script-does-not-kill-child-processes
trap 'kill $(jobs -p)' EXIT

PYTHONPATH=/opt/nvidia/jetson-gpio/lib/python:$PYTHONPATH /home/nvidia/catkin_ws/src/aruw-vision-platform-2019/scripts/shutdown-switch-watcher.py &

nvpmodel -m 0
jetson_clocks

# TODO: this is copied from the .bashrc; we should split it out into a common setup file instead.
runuser -l nvidia -c '
    export PATH=/usr/local/cuda-10.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
    export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:
    source /opt/ros/melodic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    export PYTHONPATH=/home/nvidia/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages:/usr/local/lib
    export PYTHONPATH=/home/nvidia/darknet-aruw-fast:$PYTHONPATH

    log_file=$(/home/nvidia/catkin_ws/src/aruw-vision-platform-2019/scripts/choose-log-file.py)

    VISION_NO_DEBUG= roslaunch aruw_common prod.launch 2>&1 | tee $log_file'
