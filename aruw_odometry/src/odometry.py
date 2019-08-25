#!/usr/bin/env python
import rospy
import common_utils
from common_utils import watch_for_robot_id, RobotType, register_robot_type_changed_handler
from four_wheel_mecanum_drive import SoldierDrive, HeroDrive, EngineerDrive
from sentinel_drive import SentinelDrive

class OdometryNode:
    '''
    Odometry ROS node.
    Encapsulates the DriveOdometry object that
    tracks and publishes this robot's current odometry state
    '''
    def __init__(self):
        rospy.init_node('aruw_odometry')
        common_utils.frameTransformer.init_tf()

        self.INIT_ODOMETRY_BY_ROBOT_TYPE = {
            RobotType.HERO: self.init_hero,
            RobotType.ENGINEER: self.init_engineer,
            RobotType.SOLDIER: self.init_soldier,
            RobotType.SENTINEL: self.init_sentinel
        }

        self._active_odom = None

        watch_for_robot_id()
        register_robot_type_changed_handler(self._handle_robot_type_changed)

        # Main loop
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self._active_odom:
                self._active_odom.publish()
            rate.sleep()


    def _handle_robot_type_changed(self, new_robot_type):
        '''
        Callback function for when Robot ID changes on the referee system.
        Changes the active DriveOdometry object to the one that matches the new robot type
        '''
        rospy.loginfo("New robot type set: {}; updating odometry...".format(new_robot_type))
        self._active_odom = self.INIT_ODOMETRY_BY_ROBOT_TYPE[new_robot_type]()

    def init_hero(self):
        return HeroDrive()
    
    def init_engineer(self):
        return EngineerDrive()
    
    def init_soldier(self):
        return SoldierDrive()

    def init_sentinel(self):
        return SentinelDrive()

if __name__ == '__main__':
    odometry = OdometryNode()
