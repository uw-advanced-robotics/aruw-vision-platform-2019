#!/bin/bash
set -e

sudo apt-get install libhdf5-serial-dev hdf5-tools

sudo apt-get install gfortran libopenblas-dev liblapack-dev

sudo apt-get install python-pip

sudo apt-get install zlib1g-dev zip libjpeg8-dev libhdf5-dev

sudo pip install -U numpy grpcio absl-py py-cpuinfo psutil portpicker grpcio six mock requests gast h5py astor termcolor

sudo pip install -U keras

pip install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v411 tensorflow-gpu


