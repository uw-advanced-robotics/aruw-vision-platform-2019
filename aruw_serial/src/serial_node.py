#!/usr/bin/env python

import rospy
import internal_serial
import time
from aruw_msgs.msg import AutoAimDataMessage, AlignControlMessage, AlignCompleteMessage
from serial_msg_classes import handle_receive
import struct

import numpy as np

class SerialNode:
    '''
    ROS Serial node.
    Object encapsulates all serial TX-RX transmission.
    Converts ROS messages to serial messages to be sent to the MCB and
    converts serial messages to ROS messages to be used by other nodes
    '''
    def __init__(self):
        rospy.init_node('serial')
        self.socket = internal_serial.SerialSocket(uart_callback=handle_receive)
        self.angle_listener = rospy.Subscriber("serial/turret_aim", AutoAimDataMessage, self.send_auto_aim)
        self.align_command_listener = rospy.Subscriber("serial/align_command", AlignControlMessage, self.send_align_command)
        self.align_complete_listener = rospy.Subscriber("serial/align_complete", AlignCompleteMessage, self.send_align_complete)

        while not rospy.is_shutdown(): 
            self.socket.listen()

    def send_auto_aim(self, aim_msg):
        '''
        Sends an auto aim message to the MCB over serial to control the gimbal
        @params aim_msg - AutoAimDataMessage to be sent to the MCB. Contains
                            desired pitch, yaw and whether there is target currently detected
        '''
        length = 0x05
        aim_msg_type = 0x01
        header_bytes = struct.pack("<BHBBH", self.socket.HEADER, length, 0, 0, aim_msg_type) # seq, CRC8 omitted
        body_bytes = struct.pack("<hhb", int(aim_msg.pitch * 100), int(aim_msg.yaw * 100), aim_msg.has_target)
        footer_bytes = struct.pack("<H", 0) # CRC16 omitted
        
        msg = header_bytes + body_bytes + footer_bytes
        self.socket.send_serial(msg)

    def send_align_command(self, align_control_msg):
        '''
        Sends an align control message to the MCB over serial to control engineer chassis movement
        @params align_control_msg - AlignControlMessage to be sent to the MCB. Contains
                                        desired x, y, r. Spoofs control scheme of the remote control
        '''
        length = 0x06
        align_command_msg_type = 0x02

        header_bytes = struct.pack("<BHBBH", self.socket.HEADER, length, 0, 0, align_command_msg_type) # seq, CRC8
        body_bytes = struct.pack("<hhh", align_control_msg.x, align_control_msg.y, align_control_msg.r)
        footer_bytes = struct.pack("<H", 0)
        
        msg = header_bytes + body_bytes + footer_bytes
        self.socket.send_serial(msg)

    def send_align_complete(self, align_complete_msg):
        '''
        Sends an align complete message to the MCB over serial to control engineer chassis movement
        @params align_complete_msg - AlignCompleteMessage to be sent to the MCB. Contains
                                        boolean specifying if the auto-alignment process was complete or not
        '''        
        if not align_complete_msg.complete:
            return
        
        length = 0x01
        align_complete_msg_type = 0x03

        header_bytes = struct.pack("<BHBBH", self.socket.HEADER, length, 0, 0, align_complete_msg_type) # seq, CRC8
        body_bytes = struct.pack("<B", 0x01)
        footer_bytes = struct.pack("<H", 0)
        
        msg = header_bytes + body_bytes + footer_bytes
        self.socket.send_serial(msg)

    def exit(self):
        self.socket.exit()

if __name__ == '__main__':
    SerialNode()
