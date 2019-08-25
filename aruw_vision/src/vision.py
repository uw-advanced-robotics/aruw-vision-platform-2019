#!/usr/bin/env python

import faulthandler
faulthandler.enable()
import time
import datetime

import cv2
import numpy as np
import pyrealsense2 as rs

import rospy
import tf2_ros
from std_msgs.msg import Float32

from darknet_detector import DarknetDetector
from aim_turret import AimTurret
from video_dumper import VideoDumper
from tracked_objective_controller import TrackedObjectiveController

from frame_set import VisionFrameSet
import utils
import common_utils
from common_utils import watch_for_robot_id, register_robot_type_changed_handler, RobotType, is_debug_mode

time_offset = 0.02
def set_time_offset(message):
    '''
    For tuning time_offset, the delay between the frame being
    capture on the camera and the frame data being read by
    the program.
    Not used in production.
    '''
    global time_offset
    time_offset = message.data
offset_sub = rospy.Subscriber("/time_offset", Float32, set_time_offset)

class VisionNode:
    '''
    Main Vision node.
    See our wiki for the block diagram.
    '''
    def __init__(self):
        '''
        Constructor.
        Initializes the vision processors for each robot type
        and RealSense pipeline and then starts
        running the main processing loop
        '''
        global time_offset
        
        rospy.init_node('vision')
        self._init_pipeline()

        self.VISION_PROCESSORS_BY_ROBOT_TYPE = {
            RobotType.HERO: [self.init_video_dumper, self.init_aim_turret_17mm], # TODO: 42mm and share network
            RobotType.ENGINEER: [self.init_video_dumper, self.init_engineer_alignment],
            RobotType.SOLDIER: [self.init_video_dumper, self.init_aim_turret_17mm],
            RobotType.DRONE: [self.init_video_dumper, self.init_aim_turret_17mm],
            RobotType.SENTINEL: [self.init_video_dumper, self.init_aim_turret_17mm],
        }

        #self.profile.get_device().hardware_reset()
        #self._init_pipeline()

        watch_for_robot_id()

        self._vision_processors = {
            # always start with the video dumper enabled
            self.init_video_dumper: self.init_video_dumper()
        }
        register_robot_type_changed_handler(self._handle_robot_type_changed)
        self._main_loop()

    def _main_loop(self):
        '''
        Main loop of the vision node.
        Constantly read frames from the RealSense
        and process them accordingly. 
        '''
        last_frame = 0
        frame_num = 0
        last_depth = None
        s = 0
        
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            timestamp = frames.get_color_frame().get_frame_metadata(rs.frame_metadata_value.time_of_arrival) / 1000.0 - time_offset

            #ts_fps = time.time()
            #if frame_num % 1 == 0:
                #print("FPS: " + str(1 / (ts_fps - last_frame)))
                #last_frame = ts_fps
            frame_num = frame_num + 1

            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                rospy.logerr("Error reading frame")
            
            color_frame_np = np.array(color_frame.get_data())
            depth_frame_np = np.array(depth_frame.get_data())

            frame_set = VisionFrameSet(
                color_frame_np,
                depth_frame_np,
                color_frame_np.copy() if is_debug_mode() else None,
                timestamp
            )

            if np.array_equal(depth_frame_np, last_depth):
                print("Duplicated frame at: " + str(datetime.datetime.now()))
                s = s + 1
            else:
                s = 0
            last_depth = depth_frame_np

            for processor in self._vision_processors.values():
                processor.process_frame(frame_set)

            if is_debug_mode():
                cv2.imshow("color", frame_set.debug)
                #cv2.imshow("depth", utils.make_3ch_scaled_depth(frame_set.depth))
                cv2.waitKey(1)

        for processor in self._vision_processors.values():
            processor.exit()
    
    def init_video_dumper(self):
        '''
        Initializes the video dumper video processor.
        Used as a function reference so that different robot
        types can initialize the video processor only when needed
        @return the initialized VideoDumper
        '''
        return VideoDumper()
    
    def init_engineer_alignment(self):
        return TrackedObjectiveController(self.pipeline)

    def init_aim_turret_17mm(self):
        '''
        Initializes the aim turret video processor for the 17mm turret.
        Used as a function reference so that different robot
        types can initialize the video processor only when needed
        @return the initialized AimTurret aim handler
        '''
        # TODO: specify info for the 17mm
        darknet_detector = DarknetDetector()
        return AimTurret(darknet_detector.detect_targets)
    
    def _handle_robot_type_changed(self, new_robot_type):
        '''
        Callback function on robot type changed.
        Changes the video processor from the old robot type's to
        the new robot type's, initializing new ones if needed.
        @param new_robot_type - the RobotType enum that is the new robot type
        '''
        rospy.loginfo("New robot type set: {}; updating vision processors...".format(new_robot_type))

        needed_initializers = self.VISION_PROCESSORS_BY_ROBOT_TYPE[new_robot_type]
        old_vision_processors = self._vision_processors
        self._vision_processors = {}

        for initializer in needed_initializers:
            if initializer in old_vision_processors:
                rospy.loginfo("Vision processor for {} already initialized; keeping existing instance".format(initializer.__name__))
                # bring forward all processors that are still required
                self._vision_processors[initializer] = old_vision_processors[initializer]
                del old_vision_processors[initializer]
            else:
                rospy.loginfo("Vision processor for {} doesn't already exist; initializing".format(initializer.__name__))
                # initialize new processors we don't already have
                self._vision_processors[initializer] = initializer()
        
        # kill processors which aren't needed
        for initializer, unneeded_vision_processor in old_vision_processors.items():
            rospy.loginfo("Vision processor for {} is unneeded; killing".format(initializer.__name__))
            unneeded_vision_processor.exit()

    def _init_pipeline(self):
        '''
        Sets camera and sensor parameters and then
        initializes the RealSense pipeline.
        Parameters set include frame resolutions
        and manual exposure settings and enabling 
        depth-color alignment.
        '''
        self.pipeline = rs.pipeline()

        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 60) #360
        self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
        # Recording
        #self.config.enable_record_to_file('/media/nvidia/dhy/'+str(time.time())+'.bag')

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        
        color_sensor = next(sensor for sensor in self.profile.get_device().sensors if not sensor.is_depth_sensor()) #self.profile.get_device().first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        color_sensor.set_option(rs.option.exposure, 60)
        color_sensor.set_option(rs.option.brightness, 0)
        color_sensor.set_option(rs.option.contrast, 50)
        color_sensor.set_option(rs.option.gain, 128)
        color_sensor.set_option(rs.option.gamma, 500)
        color_sensor.set_option(rs.option.saturation, 64)
        self.align = rs.align(rs.stream.color)

        # set frameTransformer state vars
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        utils.cameraInfo.set_color_intrinsics(color_profile.get_intrinsics())
        common_utils.frameTransformer.init_tf()

if __name__ == '__main__':
    VisionNode()
