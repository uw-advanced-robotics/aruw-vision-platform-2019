#!/usr/bin/env python2

import os
import Jetson.GPIO as GPIO

SWITCH_PIN = 22

if __name__ == "__main__":
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SWITCH_PIN, GPIO.IN)

        print("Waiting for switch edge...")
        GPIO.wait_for_edge(SWITCH_PIN, GPIO.FALLING)
        print("Switch triggered; shutting down...")

        os.system("shutdown -h now")
    except KeyboardInterrupt:
        GPIO.cleanup()
