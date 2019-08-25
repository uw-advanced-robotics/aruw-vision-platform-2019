#!/bin/bash
time ./darknet detector train /home/aruw/cv_data/aruw.data /home/aruw/cv_data/aruw-tiny.cfg $1 -gpus 0,1
