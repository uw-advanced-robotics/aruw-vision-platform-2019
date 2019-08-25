import os
import time
import math
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelStamped

from aruw_msgs.msg import AutoAimDataMessage, AutoAimRequestMessage
import utils
from target_tracker import TargetTracker
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from common_utils import \
    get_robot_team_color, \
    get_transform_prop, \
    qv_mult, \
    transform_to_frame, \
    convert_euler_to_quat_frame

class AimTurret:
    '''
    This is the main Turret Aim Handler.
    For each incoming camera frame, it calls the detector
    and target tracker. Then, it calculates and outputs the
    desired turret bearing for its primary target.
    '''
    def __init__(self, identify_targets):
        '''
        Constructor. Initializes required TF frame names,
        other important state variables such as keeping
        track of auto aim mode, and function definitions for
        checking if a target is valid and for target ranking
        @param identify_targets - function reference to function that takes a RGB frame
                                    as an argument and returns the detected targets in that frame
        '''
        self.identify_targets = identify_targets

        self._camera_frame = "base_camera"
        self._turret_frame = "base_turret"
        self._static_frame = "odom"
        self._base_frame = "base_turret_home"

        self._is_auto_aim_enabled = False

        def _is_valid_target(target):
            '''
            Check if target is valid i.e. its on the other team so
            we should shoot at it
            @param target - the DetectedTarget object to check
            @return True if target is valid, False otherwise
            '''
            # if we don't yet know our team color, we refuse to aim at them
            return get_robot_team_color() and target.team_color != get_robot_team_color()

        def _target_rank_fn(target):
            '''
            Key function used to compare different targets for target prioritization.
            In descending order of priority, so the lowest score will be assigned to
            the highest priority target.
            Uses a weighted sum of skew angle and distance
            @param target - the TrackedTarget object for which we are calculating
                                its key value for
            @return the calculated key value
            '''
            if not target:
                return np.finfo(np.float32).max
            ANGLE_UNIT_WEIGHT = 1/10 # being 10 degrees away is the same "negative weight" as...
            DISTANCE_UNIT_WEIGHT = 1/5 # ...being 5m away
            # TODO: factor in position covariance/std dev? Time since last seen?

            target_point_general = target.get_last_position()
            target_point_turret_rel = transform_to_frame(target_point_general, self._turret_frame)
            aim_angle_from_target = abs(
                math.degrees(
                    math.atan2(
                        math.sqrt(target_point_turret_rel.point.y**2 + target_point_turret_rel.point.z**2),
                        target_point_turret_rel.point.x
                    )
                )
            )

            target_point_base_rel = transform_to_frame(target_point_general, self._base_frame)
            distance_from_target = math.sqrt(
                (target_point_base_rel.point.x)**2
                + (target_point_base_rel.point.y)**2 
                + (target_point_base_rel.point.z)**2
            )

            return ANGLE_UNIT_WEIGHT * aim_angle_from_target + DISTANCE_UNIT_WEIGHT * distance_from_target

        self.target_tracker = TargetTracker(_is_valid_target, _target_rank_fn, self._camera_frame, self._static_frame, self._base_frame)

        self.aim_pub = rospy.Publisher('/serial/turret_aim', AutoAimDataMessage, queue_size=1, tcp_nodelay=True)
        
        #self.target_point_unfiltered_pub = rospy.Publisher('/viz/target_point_unfiltered', PointStamped, queue_size=10)
        #self.target_point_filtered_pub = rospy.Publisher('/viz/target_point_filtered', PointStamped, queue_size=10)
        self.target_point_pub = rospy.Publisher('/viz/target_point', PointStamped, queue_size=1, tcp_nodelay=True)

        self._odom_listener = rospy.Subscriber("/tf/odometry", Odometry, self._update_odom, queue_size=1, tcp_nodelay=True)
        self._acc_listener = rospy.Subscriber("/tf/accel", AccelStamped, self._update_acc, queue_size=1, tcp_nodelay=True)
        self._aim_request_listener = rospy.Subscriber("/serial/auto_aim_request", AutoAimRequestMessage, self._handle_auto_aim_request, queue_size=1, tcp_nodelay=True)

        #self._periodic_update_thread = threading.Thread(target=self._periodic_update_thread, name="AimTurretPeriodicUpdateThread")
        #self._periodic_update_thread.start()

        self.BULLET_V = 18.5
        self.BARREL_LEN = 0.19
        self._last_pitch = 90
        self._last_yaw = 90

        self._last_aim_point = None
        
        self._last_odom_rec_timestamp = rospy.get_time()
        # All translational state values in static frame (odom)
        self._our_last_pos = (0, 0, 0)
        self._our_last_vel = (0, 0, 0)
        self._our_last_acc = (0, 0, 0)
        # All angular values state values about base_center frame
        self._our_last_angvel = (0, 0, 0)
        self._our_last_angpos = (0, 0, 0, 1) # quaternion defining rotation about base_center
        self._our_last_angacc = (0, 0, 0)

    def process_frame(self, frame_set):
        '''
        Main function called to operate
        on a single incoming frame set from the camera.
        Identify targets, updates tracking and then calculates aim.
        @param frame_set - VisionFrameSet object that contains the
                                aligned color and depth frame from the camera
        '''
        # TODO: get all targets and choose based on position
        frame_targets = self.identify_targets(frame_set.color)
        target_point = self.target_tracker.update_tracking(frame_targets, frame_set, True)
        if target_point:
            #print (rospy.get_time() - target_point.header.stamp.to_sec())
            # for visualization
            self.target_point_pub.publish(target_point)
        else:
            self._last_aim_point = None
        self._update_aim(frame_set.ros_timestamp)
        if self._last_aim_point:
            frame_point = utils.deproject_ros_point_to_frame_point(self._last_aim_point, frame_set, "base_camera")
            if frame_point and frame_set.debug is not None:
                cv2.circle(frame_set.debug, utils.rounded_point(frame_point), 3, (0, 255, 255), 3)
    
    def _periodic_update_thread(self):
        # Currently disabled
        rate = rospy.Rate(500) #200
        while not rospy.is_shutdown():
            self._periodic_update(rospy.get_time())
            rate.sleep()
    
    def _periodic_update(self, timestamp):
        # Currently disabled
        self.target_tracker.periodic_update(timestamp)
        self._update_aim(timestamp)

    def _update_aim(self, timestamp):
        '''
        Calculates then publishes the aim solution calculated for the primary target if
        present, else publish the last calculated aim solution.
        @param timestamp - timestamp of latest VisionFrameSet. Currently only used for
                            visualization
        '''
        now = rospy.get_time() + 0.03
        target_state = self.target_tracker.get_predicted_primary_target_state(now)
        if target_state is None:
            # report the old angles but signal we're no longer tracking a target
            self.aim_pub.publish(AutoAimDataMessage(self._last_pitch, self._last_yaw, False))
        else:
            target_pos, target_vel, target_acc = target_state
            target_pos = (target_pos.point.x, target_pos.point.y, target_pos.point.z)
            target_vel = (target_vel.point.x, target_vel.point.y, target_vel.point.z)
            target_acc = (0, 0, 0)
            our_pos, our_vel, our_acc = self._predict_const2deriv_state(self._our_last_pos, self._our_last_vel, self._our_last_acc,
                                                                           now, self._last_odom_rec_timestamp)
            our_angpos = euler_from_quaternion(self._our_last_angpos)
            our_angpos, our_angvel, our_angacc = self._predict_const2deriv_state(our_angpos, self._our_last_angvel, self._our_last_angacc,
                                                                            now, self._last_odom_rec_timestamp)
            our_angquat = quaternion_from_euler(our_angpos[0], our_angpos[1], our_angpos[2])

            # TODO: less hacky transformation
            prop = get_transform_prop(self._base_frame, "base_center")
            x_offset = 0
            y_offset = 0
            z_offset = 0
            turret_angquat_inv = (0, 0, 0, 1)
            if prop is not None:
                (x_offset, y_offset, z_offset) = (prop[0].x,
                                                  prop[0].y,
                                                  prop[0].z)
                turret_angquat = (prop[1].x, prop[1].y, prop[1].z, prop[1].w)
                (x_offset, y_offset, z_offset) = qv_mult(turret_angquat, (x_offset, y_offset, z_offset))
                (x_offset, y_offset, z_offset) = qv_mult(our_angquat, (x_offset, y_offset, z_offset))
                turret_angquat_inv = (prop[1].x, prop[1].y, prop[1].z, -prop[1].w)
            else:
                rospy.logerr("unable to get transformation from base_center to base_frame")
            our_pos = list(our_pos)
            our_pos[0] -= x_offset
            our_pos[1] -= y_offset
            our_pos[2] -= z_offset
            our_pos = tuple(our_pos)

            our_angquat[3] = -our_angquat[3]
            pitch, yaw = self.calc_pitch_yaw(target_pos, target_vel, target_acc,
                                                our_pos, our_vel, our_acc,
                                                our_angquat, turret_angquat_inv,
                                                timestamp)
            angles = AutoAimDataMessage(pitch, yaw, True)
            self.aim_pub.publish(angles)

    def make_angles(self, turret_home_relative_point):
        '''
        Calculates pitch and yaw based on point in base frame. No predicting ahead.
        Not currently being used
        @param turret_home_relative_point - target point in base frame
        @return AutoAimDataMessage containing the calculated pitch and yaw
        '''
        #if turret_home_relative_point.header.frame_id != "base_turret_home":
            #raise Exception("bad relative frame")
        yaw = math.degrees(math.atan2(turret_home_relative_point.point.x, -turret_home_relative_point.point.y))
        pitch = math.degrees(math.atan2(math.sqrt(turret_home_relative_point.point.x**2 + turret_home_relative_point.point.y**2), -turret_home_relative_point.point.z))
        # TODO: prevent sending negative angles
        
        #with open("/home/nvidia/log.csv", "a+") as f:
        #    f.write(str(time.time()) + "," + str(pitch) + "," + str(yaw) + "\n")
        return AutoAimDataMessage(pitch, yaw, True)

    def calc_pitch_yaw(self, target_pos, target_vel, target_acc,
                        our_pos, our_vel, our_acc,
                        our_angquat_inv, turret_angquat_inv,
                        timestamp):
        '''
        Calculates pitch and yaw by predicting ahead.
        See our wiki on ballistics.
        @param target_pos - target position in static frame
        @param target_vel - target velocity in static frame
        @param target_acc - target acceleration in static frame
        @param our_pos - position of base_frame in static frame
        @param our_vel - our velocity in static frame
        @param our_acc - our acceleration in static frame
        @param our_angquat_inv - inverse of unit quaternion defining rotation from
                                    static frame to chassis frame
        @param turret_angquat_inv - inverse of unit quaternion defining rotation from
                                        chassis frame to base frame
        @param timestamp - timestamp of latest VisionFrameSet, for which this function
                                is calculating the aim solution.
        @return calculated pitch, calculated yaw
        '''
        
        # all arguments must be in ROS coordinates, and in static frame (odom)
        # tuples (x, y, z)
        # and calculated to the same timestamp
        # timestamp argument is just for visualization
        # arguments first gets converted to base frame
        self._last_aim_point = None
        x, y, z = (target_pos[0] - our_pos[0],
                    target_pos[1] - our_pos[1],
                    target_pos[2] - our_pos[2])
        vx, vy, vz = (target_vel[0] - our_vel[0],
                        target_vel[1] - our_vel[1],
                        target_vel[2] - our_vel[2])
        ax, ay, az = (target_acc[0],
                        target_acc[1],
                        target_acc[2])
        x, y, z = qv_mult(turret_angquat_inv, (x, y, z))
        vx, vy, vz = qv_mult(turret_angquat_inv, (vx, vy, vz))
        ax, ay, az = qv_mult(turret_angquat_inv, (ax, ay, az))
        x, y, z = qv_mult(our_angquat_inv, (x, y, z))
        vx, vy, vz = qv_mult(our_angquat_inv, (vx, vy, vz))
        ax, ay, az = qv_mult(our_angquat_inv, (ax, ay, az))
        relative_g = (0, 0, -9.8)
        relative_g = tuple(qv_mult(turret_angquat_inv, relative_g))
        relative_g = qv_mult(our_angquat_inv, relative_g)
        ax_target, ay_target, az_target = ax, ay, az
        ax -= relative_g[0]
        ay -= relative_g[1]
        az -= relative_g[2]
        # Solve for parametric equation of ball and target.
        # First solve for t (4th order polynomial) then put it back in for pitch and yaw
        possible_ts = np.roots(np.array([
            0.25*(ax**2 + ay**2 + az**2),
            vx*ax + vy*ay + vz*az,
            vx**2 + vy**2 + vz**2 + ax*x + ay*y + az*z - self.BULLET_V**2,
            2*(-self.BULLET_V*self.BARREL_LEN + x*vx + y*vy + z*vz),
            x**2 + y**2 + z**2 - self.BARREL_LEN**2
        ]))
        sorted_ts = np.sort(possible_ts)
        i = 0
        while sorted_ts[i] < 0 or not np.isreal(sorted_ts[i]):
            i += 1
            # IMPORTANT: DOES A POSITIVE REAL SOLUTION ALWAYS EXIST?
            if i == 4:
                print("No real positive ballistic solution found")
                print("%.2f" % x, "%.2f" % y, "%.2f" % z, "%.2f" % vx, "%.2f" % vy, "%.2f" % vz, "%.2f" % ax, "%.2f" % ay, "%.2f" % az)
                print(sorted_ts)
                return self._last_pitch, self._last_yaw
        t = sorted_ts[i]
        final_x = x + vx*t + 0.5*ax_target*(t**2)
        final_y = y + vy*t + 0.5*ay_target*(t**2)
        final_z = z + vz*t + 0.5*az_target*(t**2)
        try:
            pitch_sin_val = (z + vz*t + 0.5*az*(t**2)) / (self.BULLET_V * t + self.BARREL_LEN)
            pitch_offset_rad = math.asin(pitch_sin_val)
        except ValueError as e:
            print(e)
            pitch_sin_val = min(max(pitch_sin_val, -1), 1)
            pitch_offset_rad = math.asin(pitch_sin_val)
        final_x_hat = final_x - 0.5*relative_g[0]*(t**2)
        final_y_hat = final_y - 0.5*relative_g[1]*(t**2)
        # edge case, horizontal, final_x_hat = 0 causes math error division by 0
        if final_x_hat == 0:
            if final_y_hat < 0:
                yaw_offset_rad = -math.pi / 2
            else:
                yaw_offset_rad = math.pi / 2
        else:
            yaw_offset_rad = math.atan2(final_y_hat, final_x_hat)

        pitch_rad = math.pi / 2 + pitch_offset_rad
        yaw_rad = math.pi / 2 + yaw_offset_rad

        self._last_aim_point = utils.make_stamped_point((final_x, final_y, final_z), "base_turret_home", timestamp)
        pitch = math.degrees(pitch_rad)
        yaw = math.degrees(yaw_rad)
        self._last_pitch = pitch
        self._last_yaw = yaw

        return pitch, yaw

    def _predict_const2deriv_state(self, pos, vel, acc, target_time, src_time):
        '''
        Predicts and returns the new position, velocity and acceleration
        given constant acceleration and a time delta.
        @param pos - position. List-like
        @param vel - velocity. List-like, should have same dimensions as pos
        @param acc - accleration. List-like, should have same dimensions as pos and vel
        @param target_time - target time to predict ahead to
        @param src_time - start time where position=pos, velocity=vel, acceleration=acc
        @return predicted_position, predicted_velocity, acceleration. All tuples.
        '''
        vel_factor = target_time - src_time
        acc_factor = 0.5 * (vel_factor ** 2)
        new_pos = list(pos)
        new_vel = list(vel)
        for i in range(len(pos)):
            new_pos[i] += vel_factor * vel[i] + acc_factor * acc[i]
            new_vel[i] += vel_factor * acc[i]
        return tuple(new_pos), tuple(new_vel), tuple(acc)


    def _update_odom(self, odom_msg):
        '''
        Updates the odometry state variables with the latest
        odometry data received from the Odometry node.
        @param odom_msg - the latest Odometry message received
                            from the Odometry node. 
        '''
        self._our_last_pos = (odom_msg.pose.pose.position.x,
                                odom_msg.pose.pose.position.y,
                                odom_msg.pose.pose.position.z)
        self._our_last_vel = (odom_msg.twist.twist.linear.x,
                                    odom_msg.twist.twist.linear.y,
                                    odom_msg.twist.twist.linear.z)
        self._our_last_angpos = (odom_msg.pose.pose.orientation.x,
                                    odom_msg.pose.pose.orientation.y,
                                    odom_msg.pose.pose.orientation.z,
                                    odom_msg.pose.pose.orientation.w)
        self._our_last_angvel = (odom_msg.twist.twist.angular.x,
                                    odom_msg.twist.twist.angular.y,
                                    odom_msg.twist.twist.angular.z)
        self._last_odom_rec_timestamp = odom_msg.header.stamp.to_sec()
        
    def _update_acc(self, acc_msg):
        '''
        Updates the odometry state acceleration variables with the latest
        acceleration data received from the Odometry node.
        @param acc_msg - the latest AccelStamped message received
                            from the Odometry node. 
        '''
        self._our_last_acc = (acc_msg.accel.linear.x,
                              acc_msg.accel.linear.y,
                              acc_msg.accel.linear.z)
        self._our_last_angacc = (acc_msg.accel.angular.x,
                                acc_msg.accel.angular.y,
                                acc_msg.accel.angular.z)
    
    def _handle_auto_aim_request(self, message):
        '''
        Enables or disables auto aim based on the command received
        @param message - AutoAimRequestMessage received from controller
                            that contains the auto aim enable/disable request
        '''
        self._is_auto_aim_enabled = message.enable_auto_aim
        if message.enable_auto_aim:
            self.target_tracker.begin_tracking_primary_target()
        else:
            self.target_tracker.stop_tracking_primary_target()

    def exit(self):
        pass
