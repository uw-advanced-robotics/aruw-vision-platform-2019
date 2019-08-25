#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from aruw_msgs.msg import McbOdomMessage
from geometry_msgs.msg import AccelStamped

import math

class DriveOdometry(object):
    '''
    Base class for implementing tracking of our own robots' odometry.
    Currently all derived classes uses 2 separate Kalman Filters for this,
    one in the chassis frame, mainly for translation,
    and one in the IMU frame, mainly for rotation
    '''
    def __init__(self):
        # Publishers
        # See ROS Odometry message. Angular and translational pose, twist, and covariances
        self._odom_pub = rospy.Publisher('/tf/odometry', Odometry, queue_size=1, tcp_nodelay=True)
        # See ROS AccelStamped message. Angular and translational acceleration
        self._accel_pub = rospy.Publisher('/tf/accel', AccelStamped, queue_size=1, tcp_nodelay=True)

        # Update message listener
        self.imu_listener = rospy.Subscriber("/serial/imu", McbOdomMessage, self.handle_wrapping_and_update)

        # Time of last odometry data received/ update
        self._last_update_timestamp = None

        # For handling angle wrapping from the IMU
        # _last_* values are always raw from IMU [-pi, pi]
        self._last_recorded_angles = [0, 0, 0] # rol, pit, yaw
        self._angle_offsets = [0, 0, 0] # rol, pit, yaw

        # Init
        self._init_vars()
    
    def _init_vars(self):
        '''
        Init function
        To be overriden by derived classes
        '''
        return

    def publish(self):
        '''
        Publish our robot's current odometry state
        to tf for other nodes to use
        '''  
        timestamp = self._last_update_timestamp if self._last_update_timestamp else rospy.Time.now()
        self._publish_odom(timestamp)
        self._publish_accel(timestamp)

    def _publish_odom(self, timestamp):
        '''
        Private function. Publishes current odometry pose and twist to tf
        @see ROS Odometry message, self.publish()
        @params timestamp - timestamp of the odometry state being published
        To be overriden by derived classes
        '''
        return

    def _publish_accel(self, timestamp):
        '''
        Private function. Publishes current acceleration to tf
        @see ROS AccelStamped message, self.publish()
        @params timestamp - timestamp of the odometry state being published
        To be overriden by derived classes
        '''
        return

    def handle_wrapping_and_update(self, imu_stamped):
        '''
        Callback function upon receiving raw sensor data from the MCB over serial
        First handles the IMU's angle wrapping between -pi and pi
        to fit the linear Kalman Filters 
        @params imu_stamped - the McbOdomMessage ROS message that contains the raw
                                sensor data from the MCB
        '''
        imu_stamped = self._handle_imu_wrapping(imu_stamped)
        self.update(imu_stamped)

    def update(self, imu_stamped):
        '''
        Updates this object's Kalman filters and current odometry state
        @params imu_stamped - the McbOdomMessage ROS message that contains the raw
                                sensor data from the MCB
        To be overriden by derived classes
        '''
        return

    def _handle_imu_wrapping(self, imu_stamped):
        '''
        Unwraps the roll pitch and yaw readings from the MCB from [-pi, pi] to
        [-inf, inf]
        @params imu_stamped - the McbOdomMessage ROS message that contains the raw
                                sensor data from the MCB
        @returns the original imu_stamped message but with roll pitch and yaw
                                readings unwrapped
        '''
        corrected_angles = [0, 0, 0]
        for index, angle in [(0, imu_stamped.rol), (1, imu_stamped.pit), (2, imu_stamped.yaw)]:
            # Wrapped from -pi to pi
            if self._last_recorded_angles[index] < 0 and angle > 0:
                if (math.pi + self._last_recorded_angles[index] + math.pi - angle
                    < angle - self._last_recorded_angles[index]):
                    self._angle_offsets[index] -= 2*math.pi 
            # Wrapped from pi to -pi:
            elif self._last_recorded_angles[index]  > 0 and angle < 0:
                if (math.pi - self._last_recorded_angles[index]  + math.pi + angle
                    < self._last_recorded_angles[index] - angle):
                    self._angle_offsets[index]  += 2*math.pi
            # Update last
            self._last_recorded_angles[index] = angle
            # Add offset to account for wrapping
            corrected_angles[index] = angle + self._angle_offsets[index]
        (imu_stamped.rol, imu_stamped.pit, imu_stamped.yaw) = corrected_angles
        return imu_stamped

    def _should_reset(self):
        '''
        Checks if the Kalman Filters of this object have diverged
        and need to be reset
        @returns True if they need to be reset, False otherwise
        To be overriden by derived classes
        '''
        return False
