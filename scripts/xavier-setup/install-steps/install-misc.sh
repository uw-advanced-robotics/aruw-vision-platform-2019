#! /bin/bash
set -e

sudo apt install -y python-pip

sudo apt-get install libcanberra-gtk-module -y

pip install -U pyserial

pip install -U faulthandler

#KCF Tracker and Cython
pip install cython

cd ~
git clone https://github.com/joaofaro/KCFcpp.git
cd KCFcpp/
cmake CMakeLists.txt
make -j8

cd ~
git clone https://github.com/uoip/KCFcpp-py-wrapper.git
cd KCFcpp-py-wrapper/
sudo python setup.py install
