#!/usr/bin/env python

import serial
import struct
import sys

BAUD_RATE = 115200
PORT_NAME = "/dev/ttyTHS0"

MAX_BYTES_PER_CYCLE = 10

HEADER_LENGTH = 7

#TIMEOUT = 0.1

# Enum states for the SerialSocket
WAITING_FOR_HEADER = 0
READING_HEADER = 1
READING_DATA = 2

def debug_callback(msg_type, buffer):
    print buffer

class SerialSocket:
    '''
    Object to handle lower level interactions of the serial socket 
    '''
    def __init__(self, uart_callback=debug_callback):
        # Self-specified serial header
        self.HEADER = 0XA5
        #baudrate 115200 specified (see usart.c of MCB)
        self.port = serial.Serial(PORT_NAME,
                                  baudrate = BAUD_RATE,
                                  parity = serial.PARITY_NONE,
                                  stopbits = serial.STOPBITS_ONE,
                                  timeout = 0)
        if not self.port.is_open:
            self.port.open()
        
        # function reference to callback for processing uart data read
        self.uart_callback = uart_callback

        #VOLTAGE: 3.3V
        
        #UART PORT ON THE XAVIER IS /dev/ttyTHS0
        #MCB T <-> PIN 10 UART1_RX
        #MCB R <-> PIN 8 UART1_TX
        #MCB G <-> PIN 6 GND
        #See: https://www.jetsonhacks.com/nvidia-jetson-agx-xavier-gpio-header-pinout/ 

        #J17 6 pin uart port on the TX1/2 is /dev/ttyTHS2
        #J17 LAYOUT: RTS RX(MCB SIDE) TX(MCB SIDE) VCC CTS GND
        #Refer to http://connecttech.com/pdf/CTIM-ASG003_Manual.pdf
        #Page 12 for UART pins for carrier board
        #Using /dev/ttyTHS2 UART1 port 3/4 for carrier board
        #DO NOT use /dev/ttyS0 for the dev board. UART 0 reserved for serial debug

        # See states above
        self.state = WAITING_FOR_HEADER

        # This byte array is decoded and turned into an int upon reading first 3 bytes
        self.msg_len = 0

        # This byte array is decoded and turned into an int upon finishing reading the header
        self.msg_type = 0

    def send_serial(self, byte_string):
        #print("sent data: " + byte_string)
        self.port.write(byte_string)

    def listen(self):
        '''
        Main loop function
        Looks in the serial rx buffer and handles reading
        the bytes in there according to the state this object is in
        '''
        if self.port.in_waiting > 0:
            if self.state == WAITING_FOR_HEADER:
                byte = struct.unpack("B", self.port.read())[0]
                if byte == self.HEADER:
                    self.state = READING_HEADER

            elif self.state == READING_HEADER:
                header_bytes = self.port.read(6)
                if len(header_bytes) != 6:
                    self.state = WAITING_FOR_HEADER
                    return
                
                header = struct.unpack("<HBBH", header_bytes)
                self.msg_len = header[0]
                self.msg_type = header[-1]
                self.state = READING_DATA
            
            elif self.state == READING_DATA:
                self.uart_callback(self.msg_type, self.port.read(self.msg_len))
                self.reset_uart()

    def reset_uart(self):
        self.state = WAITING_FOR_HEADER
        self.msg_len = 0
        self.msg_type = 0

    def exit(self):
        self.port.close()
