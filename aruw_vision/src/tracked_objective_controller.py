import cv2
import rospy
import numpy as np
from PID import Remote_PID
from math import degrees
from alignment import SoldierTow
from aruw_msgs.msg import AlignCompleteMessage, AlignControlMessage, AlignRequestMessage
from nav_msgs.msg import Odometry
import pyrealsense2 as rs

class TrackedObjectiveController:
    def __init__(self, pipeline):
        self.pipeline = pipeline

        self._target_kf = None
        self._D = 3 #dynamParams
        self._M = 3 #measureParams
        self._C = 3 #controlParams

        self._last_odom_vels = None
        self._last_odom_timestamp = None
        self._last_update_timestamp = None
        self._last_filtered_point = None

        self._xpid = Remote_PID(1000.0, 0, 0)
        self._ypid = Remote_PID(1000.0, 0, 0)
        self._rpid = Remote_PID(2.0, 0, 0)

        self._align_target = None #Inactive by default
        self._epsilon = 0.5 #determine when task is complete

        self._target_ros_frame = None

        # rospy publishers and subscribers
        self._odom_listener = rospy.Subscriber('/tf/odometry', Odometry, self._update, queue_size=1)
        self._align_request_listener = rospy.Subscriber('serial/align_request', AlignRequestMessage, self._start_align_task, queue_size=1)

        self._align_control_publisher = rospy.Publisher("serial/align_command", AlignControlMessage, queue_size=1, tcp_nodelay=True)
        self._align_complete_publisher = rospy.Publisher("serial/align_complete", AlignCompleteMessage, queue_size=1, tcp_nodelay=True)

    def _start_align_task(self, align_request_msg):
        # TO-DO different align tasks
        print(align_request_msg)
        if self._align_target is not None:
            print("Alignment task already active!")
            return

        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        # Different tasks
        # All setpoints are relative to the relevant _target_ros_frame
        # that is set with _align_target
        if align_request_msg.task == 0x01:
            self._pid_setpoints(0.5, 0.0, 0)
            self._target_ros_frame = "towing_dest"
            self._align_target = SoldierTow("towing_cam", "towing_dest")

    def process_frame(self, frame_set):
        if self._align_target is None:
            # Task not active, do nothing
            return
        self._align_target.process_frame(frame_set)
        meas = self._align_target.latest_meas()
        if meas is None:
            print("Target not yet detected!")
            return
        (x, y, r) = meas
        self._correct((x, y, r))
        self.output_control()

    def _init_kf(self, initial_position):
        self._target_kf = cv2.KalmanFilter(dynamParams=self._D, measureParams=self._M, controlParams=self._C)
        # Using ROS coordinates x - ahead, y - left, z - up
        # state is relative to the chassis, so y > 0 means target is left of chassis
        # 3D state: x, y, yaw of the target relative to chassis
        # 3D measurement: x, y, yaw of the target from image processing
        # 3D control: vx, vy, vz of the chassis from odometry data
        # yaw is in degrees and degrees per second
        
        # these will be set before each predict/correct step with coefficients for the delta_t
        self._target_kf.transitionMatrix = np.eye(self._D, dtype=np.float32)
        self._target_kf.measurementMatrix = np.eye(self._D, dtype=np.float32)
        self._target_kf.controlMatrix = None # Set at each update step

        self._target_kf.processNoiseCov = None # Set at each update step
        self._target_kf.measurementNoiseCov = np.array([
            [0.05, 0, 0],
            [0, 0.05, 0],
            [0, 0, 0.5] # half a degree
        ], dtype=np.float32)

        self._target_kf.statePre = np.array(initial_position, dtype=np.float32)
        self._target_kf.statePost = np.reshape(np.copy(self._target_kf.statePre), (self._D, 1))

    def _pid_setpoints(self, x, y, r):
        self._xpid.set_setpoint(x)
        self._ypid.set_setpoint(y)
        self._rpid.set_setpoint(r)

    # ROS odom_msg callback - KF predict
    def _update(self, odom_msg):
        if self._align_target is None:
            # Task not active, do nothing
            return

        if not self._target_kf:
            self._init_kf([odom_msg.twist.twist.linear.x,
                odom_msg.twist.twist.linear.y,
                odom_msg.twist.twist.linear.z
            ])

        curr = odom_msg.header.stamp.to_sec()
        delta_t = curr - self._last_odom_timestamp if self._last_odom_timestamp else 0 
        self._target_kf.controlMatrix = np.array([
            [-delta_t, 0, 0],
            [0, -delta_t, 0],
            [0, 0, -delta_t]
        ], dtype=np.float32)
        
        self._target_kf.processNoiseCov = np.array([
            [delta_t, 0, 0],
            [0, delta_t, 0],
            [0, 0, delta_t*10]
        ], dtype=np.float32)

        '''
        print(self._target_kf.processNoiseCov.shape)
        print(self._target_kf.transitionMatrix.shape)
        print(self._target_kf.controlMatrix.shape)
        print(np.array([
                [odom_msg.twist.twist.linear.x],
                [odom_msg.twist.twist.linear.y],
                [degrees(odom_msg.twist.twist.angular.z)]
            ], dtype=np.float32).shape)
        print(self._target_kf.measurementNoiseCov.shape)
        print(self._target_kf.measurementMatrix.shape)
        print(self._target_kf.statePost.shape)
        print(self._target_kf.statePre.shape)
        '''

        self._target_kf.predict(np.array([
                [odom_msg.twist.twist.linear.x],
                [odom_msg.twist.twist.linear.y],
                [degrees(odom_msg.twist.twist.angular.z)]
            ], dtype=np.float32)
        )
        self._last_filtered_point = tuple(self._target_kf.statePre)
        self._last_odom_timestamp = curr

    def _correct(self, point):
        current_meas = np.array([
            point[0],
            point[1],
            point[2]
        ], np.float32)
        #print(current_meas.shape)
        if not self._target_kf:
            self._init_kf(current_meas)
        corrected = self._target_kf.correct(current_meas)
        self._last_filtered_point = tuple(corrected)

    def output_control(self):
        if self._align_target is None:
            # Task not active, do nothing
            return
        # Might need to predict ahead to use as current measurement idk
        x_curr, y_curr, r_curr = self._last_filtered_point
        # Note the flips here because MCB side uses different coordinate system
        # +y_out is forward, +x_out is to the right
        y_out, x_out, r_out = (-self._xpid.output(x_curr),
                               self._ypid.output(y_curr),
                               -self._rpid.output(r_curr))
        
        if ( abs(self._xpid.error()) + abs(self._ypid.error()) + abs(self._rpid.error()) < self._epsilon ):
            self.end_align_task(success=True)
        elif self._align_target.lost_target():
            self.end_align_task(success=False)
        else:
            align_control_msg = AlignControlMessage()
            align_control_msg.x = int(x_out)
            align_control_msg.y = int(y_out)
            align_control_msg.r = int(r_out)
            self._align_control_publisher.publish(align_control_msg)

    def end_align_task(self, success):
        print("Alignment task complete!")
        align_complete_msg = AlignCompleteMessage()
        align_complete_msg.complete = success
        self._align_complete_publisher.publish(align_complete_msg)
        self._align_target = None
        self._target_ros_frame = None
        
    def exit(self):
        pass

         