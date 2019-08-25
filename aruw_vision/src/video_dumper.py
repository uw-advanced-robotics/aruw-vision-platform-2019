import cv2
import time
#import h5py
import os
import errno
import re
import numpy as np
import shutil
from threading import Thread
from Queue import Queue
import rospy

SAVE_DIR = "/home/nvidia/match_recordings/"
SET_NAME = "depth_feed"

class VideoDumper:
    '''
    The VideoDumper is a vision processor class that
    saves captured video data to disk.
    Currently only saving RGB video data.
    '''
    def __init__(self):
        '''
        Constructor
        Initializes constants, state variables
        and starts and runs the video saving thread
        '''
        self.dims = (960, 540)
        #np_dims = (self.height, self.width)
        self.video_index = 0
        self.new_dir = None
        self.running = True
        self.create_main_save_dir()
        self.create_new_video_dir()
        # Number of kb to leave as buffer for the disk
        self.FREE_DISK_SPACE_BUFFER_KB = 100000
        if self.running:
            self._init_color_writer()
            '''
            self.depth_writer = h5py.File(depth_path, 'a')
            self.depth_writer.create_dataset(SET_NAME,
                                        shape=(1,)+np_dims,
                                        dtype=np.uint16,
                                        compression="gzip",
                                        compression_opts=9,
                                        chunks=True,
                                        maxshape=(None,)+np_dims)
            '''
            
            self.frame_queue = Queue()
            self.start_time = time.time()
            self.save_thread = Thread(target=self._save_thread_main)
            self.save_thread.start()

    def _init_color_writer(self):
        '''
        Initialize the VideoWriter object
        for saving color stream data to video files.
        '''
        self.video_index += 1
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        color_path = self.new_dir + '/' + str(self.video_index) + '_color.avi'
        rospy.loginfo("Opening video file {}".format(color_path))
        #depth_path = SAVE_DIR + str(time.time()) + '_depth.hdf5'
        self.color_writer = cv2.VideoWriter(color_path,
                                            fourcc,
                                            30.0,
                                            self.dims,
                                            True)

    def create_main_save_dir(self):
        '''
        Creates the main directory to save video file
        subdirectories to. 
        Stops the saving thread if it encounters an
        unexpected OS error.
        '''
        try:
            os.makedirs(SAVE_DIR)
        except OSError as e:
            # main save dir already existing is the only acceptable IO error
            if e.errno == errno.EEXIST:
                rospy.loginfo("Main save dir already exists, no further action required")
            else:
                rospy.logerr(str(e))
                self.running = False

    def create_new_video_dir(self):
        '''
        Creates a new video subdirectory in the main save directory.
        We follow a simple index incrementing naming scheme, so the
        subdirectories are named '1', '2', '3'...etc.
        Stops the saving thread if it encounters an
        unexpected OS error.
        '''
        vid_dir_index = [int(x.group(1)) for x in [re.search('saved_video_(\\d+)', dir_name) for dir_name in os.listdir(SAVE_DIR)] if x]
        vid_dir_index.append(0)
        self.new_dir = SAVE_DIR + 'saved_video_' + str(max(vid_dir_index) + 1)
        try:
            os.makedirs(self.new_dir)
            rospy.loginfo("Directory %s New video directory created", self.new_dir)
        except OSError:
            self.running = False
            rospy.logerr("Directory %s Directory already exists", self.new_dir)

    def _save_thread_main(self):
        '''
        Main video saving thread - dequeues frame data from the frame queue
        and saves it.
        Saves video data as long as no errors are encountered and
        the disk space remaining is more than the specified
        free space buffer.
        Saves videos in separate video files of at most 30 seconds each
        '''
        while self.running:
            os_info = os.statvfs(SAVE_DIR)
            free_space = (os_info.f_bavail * os_info.f_frsize) / 1024
            if free_space < self.FREE_DISK_SPACE_BUFFER_KB:
                self.running = False
                rospy.logwarn("Device running out of space, stopping saving")
                break
            next_frame_set = self.frame_queue.get()
            if next_frame_set:
                self.color_writer.write(next_frame_set.color)
            now = time.time()
            if now - self.start_time > 30:
                self.color_writer.release()
                self.start_time = now
                self._init_color_writer()
        self.color_writer.release()
        

    def process_frame(self, frame_set):
        '''
        Callback function for processing a new set of frames.
        Enqueues the color frame on the frame queue so that the
        main saving thread can dequeue and save it. 
        @param frame_set - VideoFrameSet containing frame data of the
                            latest received camera data
        '''
        if not self.running:
            return
        #color = cv2.resize(frame_set.color, self.dims)
        #start = time.time()
        if self.frame_queue.qsize() >= 2:
            rospy.logwarn("frame_queue buffer full")
            # TODO: handle exception
            self.frame_queue.get_nowait()
        self.frame_queue.put(frame_set)
        #print("Saving took " + str(time.time() - start) + "s")

        #depth = cv2.resize(frame_set.depth, self.dims)
        #self.depth_writer[SET_NAME].resize((self.depth_writer[SET_NAME].shape[0] + 1), axis=0)
        #self.depth_writer[SET_NAME][-1] = depth

    def exit(self):
        '''
        Exit function
        Joins the main save thread to exit safely
        '''
        if not self.running:
            return
        self.running = False
        self.frame_queue.put_nowait(None)
        self.save_thread.join(5)
        #self.depth_writer.close()
