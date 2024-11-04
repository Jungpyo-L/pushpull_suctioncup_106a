#!/usr/bin/env python

## ESP32 pwm generation for the suction cup experimentation

import rospy
import numpy as np
import os, sys

# serial communication
import serial
from serial import Serial

from std_msgs.msg import Int8

pwm_val = 0

def callback(data):
    global pwm_val
    pwm_val = data
    print('pwm: ', pwm_val)

def main():
    global pwm_val
    rospy.init_node('ESP32_PWM')

    rospy.Subscriber('pwm', Int8, callback)
    
    ser = serial.Serial("/dev/ttyPWM", baudrate=115200, timeout=1, write_timeout=1)
    ser.flushInput()
 
    while not rospy.is_shutdown():
        val = ser.readline().decode("utf-8")
        # print('pwm: ', val[:len(val)-1])
        pwm = str(pwm_val)
        pwm = pwm.encode("utf-8")
        ser.write(pwm + b'\n')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
