#!/usr/bin/env python
import rospy

import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from aruw_msgs.msg import AutoAimDataMessage, TurretAimFeedbackMessage
from nav_msgs.msg import Odometry
from math import radians

import common_utils
from common_utils import watch_for_robot_id, get_robot_type, RobotType

class TfBroadcasterNode:
    '''
    This ROS Node is the center for broadcasting all TF transformations
    between all TF frames by receiving all the transform data from
    the other nodes via ROS topics and then sending them to the
    TF TransformBroadcaster
    '''
    def __init__(self):
        '''
        Constructor. Registers the ROS node, initializes the
        TransformBroadcaster object and ROS subscribers to receive transform data.
        '''
        rospy.init_node('aruw_tf_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster()
        common_utils.frameTransformer.init_tf()
        watch_for_robot_id()

        self.turret_position_angles = (0, 0, 0)
        self.requested_position_angles = (0, 0, 0)
        self._odom = None
        
        self.angle_listener = rospy.Subscriber("/serial/turret_aim_feedback", TurretAimFeedbackMessage, self.update_turret_position, queue_size=1, tcp_nodelay=True)
        self.requested_angle_listener = rospy.Subscriber("/serial/turret_aim", AutoAimDataMessage, self.update_debug_requested_turret_position, queue_size=1, tcp_nodelay=True)
        self.odom_listener = rospy.Subscriber("/tf/odometry", Odometry, self.update_odom, queue_size=1, tcp_nodelay=True)

        #self.last_rec = 0
        #self.last_seq = 0
    
    def update_turret_position(self, angles_msg):
        '''
        Updates the current turret angles with the angles
        specified in the TurretAimFeedbackMessage
        @param angles_msg - the newly received TurretAimFeedbackMessage
        '''
        self.turret_position_angles = (0, 90-angles_msg.pitch, angles_msg.yaw-90) #270 + pitch yaw - 90


    def update_debug_requested_turret_position(self, angles_msg):
        '''
        Updates the current requested turret angles with the angles
        specified in the AutoAimDataMessage.
        @param angles_msg - the newly received AutoAimDataMessage
        '''
        self.requested_position_angles = (0, 270 + angles_msg.pitch, angles_msg.yaw - 90)

    def update_odom(self, odom_msg):
        '''
        Updates _odom to be the latest published odometry data.
        @param odom_msg - the Odometry message that contains the latest odometry data
        '''
        #print("TF: {}, {}, {}".format(rospy.get_time() - self.last_rec, rospy.get_time() - odom_msg.header.stamp.to_sec(), odom_msg.header.seq - self.last_seq))
        #self.last_rec = rospy.get_time()
        #self.last_seq = odom_msg.header.seq
        self._odom = odom_msg

    def _print_debug_monitor_info(self):
        '''
        For debugging: print the error between requested angles and current turret angles
        '''
        aim_error = (
            round(self.requested_position_angles[0] - self.turret_position_angles[0], 2),
            round(self.requested_position_angles[1] - self.turret_position_angles[1], 2),
            round(self.requested_position_angles[2] - self.turret_position_angles[2], 2),
        )
        print("Aim error: " + str(aim_error))

    def run(self):
        '''
        Main loop: 
        Given the current robot type,
        publish all its static transform data periodically
        '''
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            # TODO: real measurements
            # TODO: per-robot dims

            # TF TREE:
            # odom -> base_ground -> base_center -> base_link -> base_turret_home -> base_turret -> base_camera
            #                                        |---> mcb_imu       |---> base_turret_requested
            # base_ground is center of rotation about chassis
            current_robot_type = get_robot_type()

            # odom is the static world frame
            # base_ground is always flat and forward facing, translational transforms from odom only
            self.publish_odom()

            # Turret robots
            if current_robot_type in [RobotType.HERO, RobotType.SOLDIER, RobotType.DRONE, RobotType.SENTINEL]:
                # base_center is same pt as base_ground, with the rotation transformations
                # base_turret_home is at the intersection between pitch and yaw axes - the centre where line from pitch motor crosses line from yaw motor
                if current_robot_type == RobotType.SOLDIER:
                    self.publish("base_center", "base_link", (-0.260, 0.212, 0), (0, 0, 0))
                    self.publish("base_link", "mcb_imu", (0.156, -0.175, 0), (0, 0, 90))
                    self.publish("base_link", "base_turret_home", (0.260, -0.212, 0.22), (0, 0, 0))

                # TODO: base_link and mcb_imu for robots other than soldier
                elif current_robot_type == RobotType.SENTINEL:
                    # base_center is same pt as base_ground, with the rotation transformations
                    self.publish("base_center", "base_link", (0, 0, 0), (0, 0, 0))
                    self.publish("base_link", "mcb_imu", (0.156, -0.175, 0), (0, 0, 90))
                    self.publish("base_link", "base_turret_home", (0, 0, -0.2), (0, 0, 0))

                elif current_robot_type == RobotType.HERO:
                    # base_center is same pt as base_ground, with the rotation transformations
                    self.publish("base_center", "base_link", (0, 0, 0), (0, 0, 0))
                    self.publish("base_link", "mcb_imu", (0.156, -0.175, 0), (0, 0, 90))
                    self.publish("base_link", "base_turret_home", (-0.37, 0, 0.22), (0, 0, 180))

                # base_turret is the same pt as base_turret_home, but rotated to face where the turret is currently facing based on MCB feedback
                self.publish("base_turret_home", "base_turret", (0, 0, 0), self.turret_position_angles)
                # base_turret_requested is same pt as base_turret_home, but rotated to face where the turret is commanded to aim at
                self.publish("base_turret_home", "base_turret_requested", (0, 0, 0), self.requested_position_angles)
                # base_camera is position of the RealSense's RGB sensor (extreme left aperture from Realsense's perspective)
                if current_robot_type == RobotType.SOLDIER:
                    self.publish("base_turret", "base_camera", (0.03, 0.055, 0.14), (0, 2, 0))
                elif current_robot_type == RobotType.SENTINEL:
                    self.publish("base_turret", "base_camera", (0.09, -0.1, 0.045), (0, -6, 0)) 
                elif current_robot_type == RobotType.HERO:
                    self.publish("base_turret", "base_camera", (0.03, 0.055, 0.14), (0, 2, 0))
                else:
                    self.publish("base_turret", "base_camera", (0.15, 0.034 + 0.05, 0.06), (0, 0, 0)) # added 5cm to y to account for unknown offset

            # Engineer
            # TODO

            #self._print_debug_monitor_info()

            rate.sleep()

    def publish(self, parent, child, translation, rotation_euler):
        '''
        Defines a transformation property between a parent frame
        and a child frame (translation and euler rotation angles)
        and publishes it.
        @param parent - the parent TF frame
        @param child - the child TF frame
        @param translation - the linear translation data from child
                                to parent. List-like (x, y, z)
        @param rotation_euler - the euler rotation angles from child frame
                                    to parent frame. List-like (x, y, z).
                                    Values are in degrees
        '''
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        quat = tf_conversions.transformations.quaternion_from_euler(
            radians(rotation_euler[0]),
            radians(rotation_euler[1]),
            radians(rotation_euler[2]))
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.broadcaster.sendTransform(transform)

    def publish_odom(self):
        '''
        Publish the latest odometry data.
        The robot's odometry defines the transform between the static
        "odom" frame and the robot's "base_ground" frame and
        between the robot's "base_ground" frame and "base_center" frame.
        '''
        base_ground_transform = TransformStamped()
        base_center_transform = TransformStamped()
        # Set header for base_ground
        base_ground_transform.header.stamp = self._odom.header.stamp if self._odom else rospy.Time.now()
        base_ground_transform.header.frame_id = "odom"
        base_ground_transform.child_frame_id = "base_ground"
        # Set header for base_center
        base_center_transform.header.stamp = self._odom.header.stamp if self._odom else rospy.Time.now()
        base_center_transform.header.frame_id = "base_ground"
        base_center_transform.child_frame_id = "base_center"
        # Set transform properties for base_ground
        [ 
            base_ground_transform.transform.translation.x,
            base_ground_transform.transform.translation.y,
            base_ground_transform.transform.translation.z
        ] = [
            self._odom.pose.pose.position.x,
            self._odom.pose.pose.position.y,
            self._odom.pose.pose.position.z
        ] if self._odom else [0, 0, 0]
        base_ground_transform.transform.rotation = Quaternion(0, 0, 0, 1)
        # Set transform properties for base_center
        [ 
            base_center_transform.transform.translation.x,
            base_center_transform.transform.translation.y,
            base_center_transform.transform.translation.z
        ] = [0, 0, 0]
        base_center_transform.transform.rotation = self._odom.pose.pose.orientation if self._odom else Quaternion(0, 0, 0, 1)
        # Publish
        self.broadcaster.sendTransform(base_ground_transform)
        self.broadcaster.sendTransform(base_center_transform)

if __name__ == '__main__':
    broadcaster = TfBroadcasterNode()
    broadcaster.run()
