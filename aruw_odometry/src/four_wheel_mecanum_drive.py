#!/usr/bin/env python
import rospy

import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelStamped
import math
import numpy as np
from cv2 import KalmanFilter

from common_utils import convert_quat_to_frame, convert_euler_to_quat_frame, convert_angv_to_frame

from drive_odometry import DriveOdometry

class FourWheelMecanumDrive(DriveOdometry):
    '''
    Derived class of DriveOdometry.
    Tracks the odometry of robots with four wheel mecanum drives
    '''
    def __init__(self):
        super(FourWheelMecanumDrive, self).__init__()

    def _init_vars(self):
        '''
        Initialize state variables.
        All angular variables are in radians.
        All self._chassis_kf variables are relative to the static world frame ("odom")
        All self._imu_kf variables are relative to the IMU on the Type-A MCB
        '''
        # 2 Kalman Filters
        # self._chassis_kf is for chassis relative odometry
        # self._imu_kf is for imu relative odometry
        self._chassis_kf = None
        # number of state params for self._chassis_kf
        # 12-D state: x, y, z, chassis-relative yaw and their first and second derivatives in that order
        self._chassis_D = 12
        # number of measurement params for self._chassis_kf
        # 4-D measurement: vx, vy, vz, wz_wheels
        self._chassis_M = 4 
        # no control vars for self._chassis_kf

        # Mecanum wheel odom vars
        self.GEAR_RATIO = 19.0
        self.WHEEL_RADIUS = 0.075
        # To be set by derived classes, differs from robot to robot
        # Length between front mecanum wheels and back mecanum wheels
        self.L = None
        # Width between right mecanum wheels and left mecanum wheels
        self.W = None
        # Matrix that defines the transformation of wheel RPM to
        # chassis relative velocities. Differs from robot to robot
        # Since it is reliant on self.L and self.W
        self.MECANUM_M = None

        self._imu_kf = None
        # number of state params for self._imu_kf
        # 9-D state: roll, pitch, yaw and first and second derivatives in that order
        self._imu_D = 9
        # number of measurement params for self._imu_kf
        # 7-D measurement: roll, pitch, yaw and first derivatives + wz measured based on chassis measurements
        self._imu_M = 7
        # no control vars for self._imu_kf

    def _publish_odom(self, timestamp):
        '''
        See base class comment
        '''
        # One of the kalman filters have not been initialized yet, don't publish
        if not self._chassis_kf or not self._imu_kf:
            return

        if self._should_reset():
            rospy.logwarn("Odometry diverged, resetting kalman filters...")
            self._reset_kf()
            return

        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_center" # fitting 2 frame transforms into 1 odom msg
        odom_msg.header.stamp = timestamp
        chassis_state = self._chassis_kf.statePost[:, 0]
        imu_state = self._imu_kf.statePost[:, 0]

        # Translation
        (odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z) = chassis_state[0:3]

        # Rotation
        mod = 2*math.pi
        mcb_imu_quat = tf_conversions.transformations.quaternion_from_euler(
            imu_state[0] % mod,
            imu_state[1] % mod,
            imu_state[2] % mod
        )
        pose_quat = convert_quat_to_frame(mcb_imu_quat, "base_center", "mcb_imu")
        (odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w) = pose_quat

        # Set pose covariances
        chassis_cov = self._chassis_kf.errorCovPost
        imu_cov = self._imu_kf.errorCovPost

        pose_cov_matrix = np.zeros((6,6), dtype=np.float32)
        pose_cov_matrix[0:3,0:3] = chassis_cov[0:3,0:3]
        pose_cov_matrix[3:6,3:6] = imu_cov[0:3,0:3]
        odom_msg.pose.covariance = tuple(pose_cov_matrix.flatten())

        # Translational velocity
        (odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z) = chassis_state[4:7]

        # Angular velocities
        twist_euler = convert_angv_to_frame(imu_state[3:6], "base_center", "mcb_imu")
        (odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z) = twist_euler[0:3]

        # Set velocity covariances
        twist_cov_matrix = np.zeros((6,6), dtype=np.float32)
        twist_cov_matrix[0:3,0:3] = chassis_cov[4:7,4:7]
        twist_cov_matrix[3:6,3:6] = imu_cov[3:6,3:6]
        odom_msg.twist.covariance = tuple(twist_cov_matrix.flatten())

        self._odom_pub.publish(odom_msg)

    def _publish_accel(self, timestamp):
        '''
        See base class comment
        '''
        # One of the kalman filters have not been initialized yet, don't publish
        if not self._imu_kf or not self._chassis_kf:
            return
        accel_msg = AccelStamped()
        accel_msg.header.frame_id = "odom"
        accel_msg.header.stamp = timestamp

        (accel_msg.accel.linear.x,
            accel_msg.accel.linear.y,
            accel_msg.accel.linear.z) = self._chassis_kf.statePost[8:11, 0]
        
        (accel_msg.accel.angular.x,
            accel_msg.accel.angular.y,
            accel_msg.accel.angular.z) = self._imu_kf.statePost[6:9, 0]

        self._accel_pub.publish(accel_msg)

    def _init_chassis_kf(self, initial_meas):
        '''
        Initializes self._chassis_kf to 0 position and acceleration and initial
        velocity based on the initial measurements.
        @params initial_meas - (4, 1) np.array world frame relative [[vx], [vy], [vz], [wz_chassis]]
        '''
        self._chassis_kf = KalmanFilter(dynamParams=self._chassis_D, measureParams=self._chassis_M)

        # transitionMatrix will be edited before each predict/correct step with coefficients for the delta_t
        self._chassis_kf.transitionMatrix = np.eye(self._chassis_D, dtype=np.float32) # transitionMatrix @ state_n = state_n+1
        self._chassis_kf.measurementMatrix = np.array([ # predictedMeasurements += measurementMatrix @ statePre_n
            [0,   0,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0], # vx
            [0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   0,   0], # vy
            [0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   0], # vz
            [0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0], # wz_wheels
        ], np.float32)

        self._chassis_kf.processNoiseCov = np.diag(np.array([
            0.05 **2,
            0.05 **2,
            0.05 **2,
            0.05 **2,
            0.1 **2,
            0.1 **2,
            0.1 **2,
            0.1 **2,
            0.01 **2,
            0.01 **2,
            0.01 **2,
            0.01 **2
        ], dtype=np.float32))
        self._chassis_kf.measurementNoiseCov = 0.005 * np.eye(self._chassis_M, dtype=np.float32)

        # initial state
        # opencv KalmanFilter initializes both statePre and statePost to 0 proper vectors (i.e. 2 dims)
        vel_start = 4
        vel_end = vel_start + self._chassis_M
        self._chassis_kf.statePre[vel_start:vel_end, 0] = initial_meas[:self._chassis_M, 0]
        self._chassis_kf.statePost[vel_start:vel_end, 0] = initial_meas[:self._chassis_M, 0]

    def _init_imu_kf(self, initial_meas):
        '''
        Initializes self._imu_kf to 0 angular position and acceleration and initial
        velocity based on the initial measurements.
        @params initial_meas - (7, 1) np.array IMU frame relative
                                [[roll], [pitch], [yaw], [wx], [wy], [wz], [yaw based on chassis yaw]]
        '''
        self._imu_kf = KalmanFilter(dynamParams=self._imu_D, measureParams=self._imu_M)

        # transitionMatrix will be edited before each predict/correct step with coefficients for the delta_t
        self._imu_kf.transitionMatrix = np.eye(self._imu_D, dtype=np.float32) # transitionMatrix @ state_n = state_n+1
        self._imu_kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
        ], dtype=np.float32)

        # guessed covariances. 
        self._imu_kf.processNoiseCov = np.diag(np.array([
            0.05 **2,
            0.05 **2,
            0.05 **2,
            0.1 **2,
            0.1 **2,
            0.1 **2,
            0.01 **2,
            0.01 **2,
            0.01 **2
        ], dtype=np.float32))
        self._imu_kf.measurementNoiseCov = np.diag(np.array([
            0.005 **2,
            0.005 **2,
            0.05 **2,
            0.01 **2,
            0.01 **2,
            0.01 **2,
            0.0075 **2
        ], dtype=np.float32))

        # initial state
        self._imu_kf.statePre[:self._imu_M, 0] = initial_meas[:self._imu_M, 0]
        self._imu_kf.statePost[:self._imu_M, 0] = initial_meas[:self._imu_M, 0]

    def _configure_chassis_kf_for_update(self, delta_t):
        '''
        Configures self._chassis_kf for the predict step based on the time 
        difference between last measurement and current measurement
        as part of the extended Kalman Filter
        @params delta_t. Time difference between last measurement and current measurement
        '''
        vel = delta_t # velocity/2nd-derivative factor
        acc = 0.5 * delta_t**2
        np.fill_diagonal(self._chassis_kf.transitionMatrix[:8, 4:], vel)
        np.fill_diagonal(self._chassis_kf.transitionMatrix[:4, 8:], acc)

    def _configure_imu_kf_for_update(self, delta_t):
        '''
        Configures self._imu_kf for the predict step based on the time 
        difference between last measurement and current measurement
        as part of the extended Kalman Filter
        @params delta_t. Time difference between last measurement and current measurement
        '''
        vel = delta_t # velocity/2nd-derivative factor
        acc = 0.5 * delta_t**2
        np.fill_diagonal(self._imu_kf.transitionMatrix[:6, 3:], vel)
        np.fill_diagonal(self._imu_kf.transitionMatrix[:3, 6:], acc)

    def update(self, imu_stamped):
        '''
        Updates the current odometry state of this object by
        carrying out the predict and update step on self._chassis_kf
        and self._imu_kf
        @params imu_stamped - the McbOdomMessage ROS message that contains the raw
                                sensor data from the MCB
        '''
        current_timestamp = imu_stamped.header.stamp.to_sec()
        # Must update chassis first and then IMU
        # bec imu KF's yaw_wheels measurement uses 
        # chassis statePost's yaw_wheels estimate to compute
        # an additional measurement for yaw based on wheels yaw to correct IMU yaw drift

        # Chassis update
        # Convert from RPM to radians/s
        # See: https://dacemirror.sci-hub.tw/journal-article/7eee4f02b97da1bba632b6e57aa7856e/conduraru2014.pdf
        rf = -(imu_stamped.rf_rpm / self.GEAR_RATIO * 2 * math.pi) / 60 
        lf = (imu_stamped.lf_rpm / self.GEAR_RATIO * 2 * math.pi) / 60
        lb = (imu_stamped.lb_rpm / self.GEAR_RATIO * 2 * math.pi) / 60
        rb = -(imu_stamped.rb_rpm /self.GEAR_RATIO * 2 * math.pi) / 60
        velocities = np.matmul(self.MECANUM_M, np.array([
            [rf],
            [lf],
            [lb],
            [rb]
        ]))
        # Sum and convert to world relative
        # now working in ros coordinates
        vx_chassis = velocities[1,0]
        vy_chassis = -velocities[0,0]
        # Chassis predict: if KF init already, use statePre's yaw to compute
        # robot vx and vy in static frame
        # else use chassis_yaw = 0
        # Stretch TODO: slope odometry
        chassis_yaw = 0
        if self._chassis_kf:
            self._configure_chassis_kf_for_update(current_timestamp - self._last_update_timestamp.to_sec() if self._last_update_timestamp else 0)
            self._chassis_kf.predict()
            chassis_yaw = self._chassis_kf.statePre[3,0]
        vx = vx_chassis * math.cos(chassis_yaw) - vy_chassis * math.sin(chassis_yaw)
        vy = vx_chassis * math.sin(chassis_yaw) + vy_chassis * math.cos(chassis_yaw)
        wz_wheels = velocities[2, 0]
        current_chassis_meas = np.array([
            [vx],
            [vy],
            [0],
            [wz_wheels]
        ], np.float32)

        if not self._chassis_kf:
            self._init_chassis_kf(current_chassis_meas)
            self._configure_chassis_kf_for_update(0)
            self._chassis_kf.predict()

        self._chassis_kf.correct(current_chassis_meas)

        # IMU update
        current_imu_meas = np.array([
            [imu_stamped.rol],
            [imu_stamped.pit],
            [imu_stamped.yaw],
            [imu_stamped.wx],
            [imu_stamped.wy],
            [imu_stamped.wz],
            [0]
        ], np.float32)
        if not self._imu_kf:
            self._init_imu_kf(current_imu_meas)

        self._configure_imu_kf_for_update(current_timestamp - self._last_update_timestamp.to_sec() if self._last_update_timestamp else 0)
        self._imu_kf.predict()
        # Convert yaw wheels readings from base_center frame to IMU frame
        # to use as an additional measurement for the yaw in order to correct IMU yaw drift
        predicted_trans_quat = tf_conversions.transformations.quaternion_from_euler(self._imu_kf.statePre[0, 0],
                                                                                    self._imu_kf.statePre[1, 0],
                                                                                    self._imu_kf.statePre[2, 0])
        predicted_trans_quat[3] = -predicted_trans_quat[3]
        yaw_wheels = convert_euler_to_quat_frame((0, 0, self._chassis_kf.statePost[3,0]), predicted_trans_quat)[2]
        current_imu_meas[6, 0] = yaw_wheels
        #print(current_imu_meas)
        self._imu_kf.correct(current_imu_meas)
        #print(self._imu_kf.statePost[2,0], self._imu_kf.statePost[5, 0], self._imu_kf.statePost[8, 0])
        #with open("/home/nvidia/our-log.csv", "a+") as f:
            #f.write(str(current_timestamp) + "," + str(self._chassis_kf.statePost[0, 0]) + "," + str(self._chassis_kf.statePost[1, 0]) + "," + str(self._chassis_kf.statePost[2, 0]) + "\n")


        self._last_update_timestamp = imu_stamped.header.stamp

    def _should_reset(self):
        '''
        See base class comment
        '''
        if self._imu_kf and self._chassis_kf:
            abs_imu_state = np.abs(self._imu_kf.statePost)
            abs_chassis_state = np.abs(self._chassis_kf.statePost)
            abs_imu_error = np.abs(self._imu_kf.errorCovPost)
            abs_chassis_error = np.abs(self._chassis_kf.errorCovPost)
            return (np.any(np.isnan(abs_imu_error))
                    or np.any(np.isnan(abs_chassis_error))
                    or np.any(abs_imu_state[3:, :] > 100)
                    or np.any(abs_chassis_state[4:, :] > 100))
        return False

    def _reset_kf(self):
        '''
        Clears the 2 Kalman Filters' states
        '''
        self._chassis_kf = None
        self._imu_kf = None

class SoldierDrive(FourWheelMecanumDrive):
    '''
    Derived class of FourWheelMecanumDrive
    For soldier
    '''
    def __init__(self):
        super(SoldierDrive, self).__init__()
    def _init_vars(self):
        super(SoldierDrive, self)._init_vars()
        self.L = 0.3475
        self.W = 0.41988
        rot_factor = 2 / (self.L + self.W)
        self.MECANUM_M = (self.WHEEL_RADIUS / 4) * np.array([
            [-1, 1, -1, 1],
            [1, 1, 1, 1,],
            [rot_factor, -rot_factor, -rot_factor, rot_factor]
        ])

class HeroDrive(FourWheelMecanumDrive):
    '''
    Derived class of FourWheelMecanumDrive
    For hero
    '''
    def __init__(self):
        super(HeroDrive, self).__init__()
    def _init_vars(self):
        super(HeroDrive, self)._init_vars()
        self.L = 0.5
        self.W = 0.5
        rot_factor = 2 / (self.L + self.W)
        self.MECANUM_M = (self.WHEEL_RADIUS / 4) * np.array([
            [-1, 1, -1, 1],
            [1, 1, 1, 1,],
            [rot_factor, -rot_factor, -rot_factor, rot_factor]
        ])

class EngineerDrive(FourWheelMecanumDrive):
    '''
    Derived class of FourWheelMecanumDrive
    For engineer
    '''
    def __init__(self):
        super(EngineerDrive, self).__init__()
    def _init_vars(self):
        super(EngineerDrive, self)._init_vars()
        # TO-DO: Get actual measurements from CAD
        self.L = 0.5
        self.W = 0.5
        rot_factor = 2 / (self.L + self.W)
        self.MECANUM_M = (self.WHEEL_RADIUS / 4) * np.array([
            [-1, 1, -1, 1],
            [1, 1, 1, 1,],
            [rot_factor, -rot_factor, -rot_factor, rot_factor]
        ])
