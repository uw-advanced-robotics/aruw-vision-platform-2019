# common utilities file symlinked into all dependent nodes, src is in aruw_common

import math
import os

import rospy
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_from_euler, euler_from_quaternion, unit_vector

from aruw_msgs.msg import RobotIdMessage

class TeamColor:
    '''
    Enum class
    '''
    RED = 'red'
    BLUE = 'blue'

class RobotType:
    '''
    Enum class
    '''
    HERO = 'hero'
    ENGINEER = 'engineer'
    SOLDIER = 'soldier'
    DRONE = 'drone'
    SENTINEL = 'sentinel'

# List maps (ID index - 1)to RobotType
ROBOT_TYPE_ID_MAP = [
    RobotType.HERO,
    RobotType.ENGINEER,
    RobotType.SOLDIER,
    RobotType.SOLDIER,
    RobotType.SOLDIER,
    RobotType.DRONE,
    RobotType.SENTINEL
]

_robot_id_listener = None
def watch_for_robot_id():
    '''
    Create a new listener to watch for updates to Robot ID
    '''
    global _robot_id_listener
    if not _robot_id_listener:
        _robot_id_listener = rospy.Subscriber("/serial/robot_id", RobotIdMessage, on_robot_id_received)

def register_team_color_changed_handler(handler):
    '''
    Registers a new callback function to call when
    team has changed.
    @param handler - function reference to the new callback function
    '''
    global _team_color, _team_color_changed_handlers
    _team_color_changed_handlers.append(handler)
    if _team_color:
        handler(_team_color)

def register_robot_type_changed_handler(handler):
    '''
    Registers a new callback function to call when
    robot type has changed.
    @param handler - function reference to the new callback function
    '''
    global _robot_type, _robot_type_changed_handlers
    _robot_type_changed_handlers.append(handler)
    if _robot_type:
        handler(_robot_type)

_team_color_changed_handlers = []
_robot_type_changed_handlers = []
_team_color = None
_robot_type = None
def on_robot_id_received(robot_id_message):
    '''
    Overall callback function upon receiving new RobotIdMessage.
    Determines what the new robot type and team color is based on
    the robot ID and calls the registered callback functions.
    @param robot_id_message - the newly received RobotIdMessage
    '''
    global _team_color, _team_color_changed_handlers
    global _robot_type, _robot_type_changed_handlers

    robot_id_val = robot_id_message.robot_id

    if 1 <= robot_id_val <= 7:
        new_team_color = TeamColor.RED
        new_robot_type = ROBOT_TYPE_ID_MAP[robot_id_val - 1]
    elif 11 <= robot_id_val <= 17:
        new_team_color = TeamColor.BLUE
        new_robot_type = ROBOT_TYPE_ID_MAP[robot_id_val - 11]
    else:
        # invalid value...
        rospy.logerr("Invalid robot id \"{}\" received".format(robot_id_val))
        return

    if _team_color != new_team_color:
        _team_color = new_team_color
        for handler in _team_color_changed_handlers:
            handler(_team_color)

    if _robot_type != new_robot_type:
        _robot_type = new_robot_type
        for handler in _robot_type_changed_handlers:
            handler(_robot_type)

def get_robot_team_color():
    '''
    Getter for current robot team color
    @returns current team color
    '''
    global _team_color
    return _team_color

def get_robot_type():
    '''
    Getter for current robot type
    @returns current robot type
    '''
    global _robot_type
    return _robot_type

# TF stuff
class FrameTransformer(object):
    '''
    This object handles transforming between ROS frames
    for all other nodes of our program. Each separate 
    ROS node will have a instance of this object running if required.
    '''
    def __init__(self):
        '''
        Constructor. Initializes fields to None.
        '''
        self._tf_buffer = None
        self._tf_listener = None
    def init_tf(self):
        '''
        Initializer
        '''
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
    def get_tf_buffer(self):
        '''
        Getter for TF buffer
        @return this instance's TF buffer
        '''
        return self._tf_buffer

frameTransformer = FrameTransformer()

def transform_to_frame(point, target_frame):
    '''
    Transform a ROS point from its frame (specified in its header)
    to a target frame
    @param point - the ROS point to be transformed. Should have a complete header and its frame ID
    @param target_frame - the target TF frame to transform the point into
    @return the transformed ROS point if successful, None otherwise
    '''
    global frameTransformer
    try:
        return frameTransformer.get_tf_buffer().transform(point, target_frame)
    except tf2_ros.ExtrapolationException as e:
        err_msg = "Failed to look up transform from {} to {}".format(point.header.frame_id, target_frame)
        rospy.logerr(err_msg + ", " + str(e))
        return None

def get_transform_prop(target_frame, src_frame):
    '''
    Returns the transformation (translation and rotation) from the 
    src frame to the target frame.
    @param target_frame - the target TF frame
    @param src_frame - the source TF frame
    @return (translation, rotation) if successful, None otherwise.
            translation is a ROS point object while
            rotation is a ROS quaternion object
    '''
    global frameTransformer
    try:
        trans = frameTransformer.get_tf_buffer().lookup_transform(target_frame, src_frame,
                frameTransformer.get_tf_buffer().get_latest_common_time(target_frame, src_frame))
        return trans.transform.translation, trans.transform.rotation
    except tf2_ros.TransformException as e:
        err_msg = "Failed to look up transform matrices from {} to {}".format(src_frame, target_frame)
        rospy.logerr(err_msg + ", " + str(e))
        return None

def convert_quat_to_frame(quat, target_frame, src_frame):
    '''
    Convert a quaternion from the source frame to the target frame
    @param quat - the quaternion to be converted. Tuple (x, y, z, w)
    @param target_frame - the target TF frame to be converted into
    @param src_frame - the source TF frame of the quaternion
    @return the converted quaternion, tuple (x, y, z, w) if successful, the original quaternion otherwise
    '''
    trans = get_transform_prop(target_frame, src_frame)
    if trans is None:
        rospy.logerr("Failed to convert quaternion from {} to {}".format(src_frame, target_frame))
        return quat
    _, transform_quat = trans
    # Note inverse
    transform_quat_data = (transform_quat.x, transform_quat.y, transform_quat.z, -transform_quat.w)
    return convert_quat_to_quat_frame(quat, transform_quat_data)

def convert_quat_to_quat_frame(quat, relative_trans_quat):
    '''
    Given the quaternion that defines the transformation from its frame of reference
    to the target frame of reference, transform a quaternion to the target frame
    @param quat - the quaternion to be transformed. List-like (x, y, z, w)
    @param relative_trans_quat - the quaternion that defines the transformation from
                                    quat's frame to the target frame
    @return the converted quaternion, tuple (x, y, z, w)
    '''
    # We want to transform quat from Frame1 to Frame2
    # then if Frame1 --q-> Frame2, relative_trans_quat is q-inverse
    relative_trans_quat = unit_vector(relative_trans_quat)
    relative_trans_inv = (relative_trans_quat[0], relative_trans_quat[1], relative_trans_quat[2], -relative_trans_quat[3])
    x_qinv = unit_vector(quaternion_multiply(quat, relative_trans_inv))
    return unit_vector(quaternion_multiply(relative_trans_quat, x_qinv))

def convert_euler_to_quat_frame(euler, relative_trans_quat):
    '''
    Given the quaternion that defines the transformation from its frame of reference
    to the target frame of reference, transform a triplet of euler rotation angles 
    to the target frame
    @param euler - the euler angles to be converted. List-like (x, y, z)
    @param relative_trans_quat - the quaternion that defines the transformation from
                                    euler's frame to the target frame.
    @see convert_quat_to_quat_frame
    @return the converted euler angles, tuple (x, y, z)
    '''
    quat = quaternion_from_euler(euler[0], euler[1], euler[2])
    return euler_from_quaternion(convert_quat_to_quat_frame(quat, relative_trans_quat))

def convert_euler_to_frame(euler, target_frame, src_frame):
    '''
    Converts a triplet of euler rotation angles in a source frame
    to a target frame
    @param euler - the euler angles to be converted. List-like (x, y, z)
    @param target_frame - the target TF frame to be converted into
    @param src_frame - the source TF frame that euler came from
    @return the converted euler angles, tuple (x, y, z) if successful,
                the original euler angles otherwise.
    '''
    quat = quaternion_from_euler(euler[0], euler[1], euler[2])
    result = convert_quat_to_frame(quat, target_frame, src_frame)
    return euler_from_quaternion(result)

def rot_angles(angles, rot):
    '''
    Determine the new values for a triplet of euler rotation
    angles in a rotated frame of reference.
    Used to convert angular velocity and acceleration between frames.
    @param angles - the euler angles to be transformed. List-like (x, y, z)
    @param rot - the euler angles that define the transformation from
                    angles' source frame to its target frame
    @return the converted euler triplet
    '''
    siny = math.sin(rot[1])
    cosy = math.cos(rot[1])
    sinz = math.sin(rot[2])
    cosz = math.cos(rot[2])
    xv = cosy*cosz*angles[0] + sinz*angles[1]
    yv = -cosy*sinz*angles[0] + cosz*angles[1]
    zv = siny*angles[1] + angles[2]
    return (xv, yv, zv)

def convert_angv_to_frame(angles, target_frame, src_frame):
    '''
    Converts angular velocity or acceleration from a source frame to a target frame
    @param angles - angular velocity or acceleration about axis (x, y, z). List-like (x, y, z)
    @param target_frame - the target TF frame to convert to
    @param src_frame - the src TF frame that angles is in
    @return the converted set of angles if successful, the original angles otherwise
    '''
    trans = get_transform_prop(target_frame, src_frame)
    if not trans:
        return angles
    _, quat = trans
    rot = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
    return rot_angles(angles, rot)

def convert_linav_to_frame(aorv, target_frame, src_frame):
    '''
    Converts linear acceleration or velocity from a source TF frame
    to a target TF frame by rotating it according to the relative transform
    @param aorv - the linear acceleration of velocity to be converted. List-like (x, y, z)
    @param target_frame - the target TF frame
    @param src_frame - the src TF frame that aorv is in
    @return the converted linear acceleration or velocity values, tuple (x, y, z), if successful,
                the original aorv otherwise
    '''
    transform = get_transform_prop(target_frame, src_frame)
    if transform is None:
        return aorv
    translation, quat = transform
    quat_data = (quat.x, quat.y, quat.z, quat.w)
    return qv_mult(quat_data, aorv)

def qv_mult(q1, v1):
    '''
    Rotates a vector by a unit quaternion
    @param q1 - the quaternion to rotate the vector by. Tuple (x, y, z, w)
    @param v1 - the vector to be rotated. Tuple (x, y, z)
    @return the rotated vector
    '''
    q2 = v1 + (0.0,)
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[:3]

def is_debug_mode():
    '''
    Check if the program is currently being ran in debug mode
    @return True if program is currently being ran in debug mode, False otherwise
    '''
    return "VISION_NO_DEBUG" not in os.environ