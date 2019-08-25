import rospy
from aruw_msgs.msg import TurretAimFeedbackMessage, McbOdomMessage, AlignRequestMessage, RobotIdMessage, AutoAimRequestMessage
from std_msgs.msg import Header
import time
import struct
import numpy as np
import math

'''
This file contains the various serial feedback handler classes.
The serial feedback handler classes are called as callback 
functions for their respective serial message types
to process the data into their respective ROS messages to be
forwarded for use by other nodes
'''

class TurretFeedbackHandler:
    '''
    Handles turret gimbal feedback - pitch and yaw angles
    '''
    def __init__(self):
        self.feedback_pub = rospy.Publisher("serial/turret_aim_feedback", TurretAimFeedbackMessage, queue_size=1, tcp_nodelay=True)

    def __call__(self, buffer):
        #print " ".join(x.encode("hex") for x in buffer)
        try:
            data = struct.unpack("<hh", buffer)
        except struct.error as e:
            rospy.logerr("Invalid turret feedback message")
            #print " ".join(x.encode("hex") for x in buffer)
            return
        feedback = TurretAimFeedbackMessage()
        feedback.pitch = float(data[0]) / 100
        feedback.yaw = float(data[1]) / 100
        #print(feedback)
        self.feedback_pub.publish(feedback)

class ImuFeedbackHandler:
    '''
    Handles odometry feedback - IMU and drive motor readings
    '''
    def __init__(self):
        self.feedback_pub = rospy.Publisher("serial/imu", McbOdomMessage, queue_size=1, tcp_nodelay=True)
        #self.last_published = 0

    def __call__(self, buffer):
        #print " ".join(x.encode("hex") for x in buffer)
        try:
            data = struct.unpack("<hhhhhhhhhhhhh", buffer)
        except struct.error as e:
            #print(" ".join(x.encode("hex") for x in buffer))
            rospy.logerr("Invalid imu feedback message: " + str(e) + ", length of buf: " + str(len(buffer)))
            return
        #print(" ".join(x.encode("hex") for x in buffer))
        #print(data)
        # Create ROS msg
        imu = McbOdomMessage()
        # Timestamp header
        header = Header()
        header.stamp = rospy.Time(rospy.get_time())
        header.frame_id = "mcb_imu"
        imu.header = header
        # Dump data
        imu.ax = float(data[0]) / 100
        imu.ay = float(data[1]) / 100
        imu.az = float(data[2]) / 100
        imu.rol = math.radians(float(data[3]) / 100)
        imu.pit = math.radians(float(data[4]) / 100)
        imu.yaw = math.radians(float(data[5]) / 100)
        imu.wx = float(data[6]) / 100
        imu.wy = float(data[7]) / 100
        imu.wz = float(data[8]) / 100
        imu.rf_rpm = data[9]
        imu.lf_rpm = data[10]
        imu.lb_rpm = data[11]
        imu.rb_rpm = data[12]
        self.feedback_pub.publish(imu)

class AlignRequestHandler:
    '''
    Handles auto alignment requests - type of auto alignment task
    '''
    def __init__(self):
        self.request_pub = rospy.Publisher("serial/align_request", AlignRequestMessage, queue_size=1, tcp_nodelay=True)

    def __call__(self, buffer):
        try:
            data = struct.unpack("<B", buffer)
        except struct.error as e:
            rospy.logerr("Invalid align control message")
            return
        #print " ".join(x.encode("hex") for x in buffer)
        request = AlignRequestMessage()
        request.task = data[0]
        #print(request)
        self.request_pub.publish(request)

class RobotIdHandler:
    '''
    Handles robot ID messages - tells us which robot this robot currently is
    '''
    def __init__(self):
        self.robot_id_pub = rospy.Publisher("/serial/robot_id", RobotIdMessage, queue_size=1, tcp_nodelay=True, latch=True)

    def __call__(self, buffer):
        #print(" ".join(x.encode("hex")) for x in buffer)
        try:
            data = struct.unpack("<B", buffer)
        except struct.error as e:
            rospy.logerr("Invalid robot ID message")
            return

        robot_id = RobotIdMessage()
        robot_id.robot_id = data[0]
        #print(robot_id)
        self.robot_id_pub.publish(robot_id)


class AutoAimRequestHandler:
    '''
    Handles auto aim request messages
    '''
    def __init__(self):
        self.request_pub = rospy.Publisher("/serial/auto_aim_request", AutoAimRequestMessage, queue_size=1, tcp_nodelay=True)

    def __call__(self, buffer):
        try:
            data = struct.unpack("<B", buffer)
        except struct.error as e:
            rospy.logerr("Invalid auto aim control message")
            return

        request = AutoAimRequestMessage()
        request.enable_auto_aim = data[0]
        self.request_pub.publish(request)

# dictionary for mapping serial message type to its appropriate handler
receive_handlers = {
    0x01: TurretFeedbackHandler(),
    0x02: ImuFeedbackHandler(),
    0x03: AlignRequestHandler(),
    0x04: RobotIdHandler(),
    0x05: AutoAimRequestHandler()
}

def handle_receive(message_type, buffer):
    if message_type not in receive_handlers:
        rospy.logwarn_throttle(30, "Unhandled message type {}".format(message_type))
        return

    receive_handlers[message_type](buffer)

