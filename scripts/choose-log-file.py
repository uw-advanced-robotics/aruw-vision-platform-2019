#!/usr/bin/python

import sys
import os
import errno
import re

LOG_DIR = "/home/nvidia/vision_logs/"

if __name__ == "__main__":
    try:
        os.makedirs(LOG_DIR)
    except OSError as e:
        # log dir already existing is the only acceptable IO error
        if e.errno != errno.EEXIST:
            raise

    vid_dir_index = [int(x.group(1)) for x in [re.search('vision_log_(\\d+).txt', dir_name) for dir_name in os.listdir(LOG_DIR)] if x]
    vid_dir_index.append(0)
    log_file_path = LOG_DIR + 'vision_log_' + str(max(vid_dir_index) + 1) + ".txt"

    print (log_file_path)
            
