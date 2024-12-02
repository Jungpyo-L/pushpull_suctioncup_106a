#!/usr/bin/env python

import rospy
import numpy as np
import os, sys

# serial communication
import serial
from serial import Serial

from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull

pwm_val = 0
state_val = 0

def callback(data):
    global pwm_val
    global state_val
    pwm_val = data.pwm
    state_val = data.state
    print('state: ', state_val)
    print('pwm: ', pwm_val)

def main():
    global pwm_val
    global state_val
    rospy.init_node('ESP32_PWM')

    rospy.Subscriber('PushPull', PushPull, callback)
    
    ser = serial.Serial("/dev/ttyPWM", baudrate=115200, timeout=1, write_timeout=1)
    ser.flushInput()
 
    while not rospy.is_shutdown():
        val = ser.readline().decode("utf-8")
        # Print received value
        # print('Received from serial: ', val.strip())

        # Prepare the message to be sent
        state = str(state_val)
        pwm = str(pwm_val)
        message = f"{state}_{pwm}\n".encode("utf-8")  # Encode as bytes

        # Send the message to the serial port
        ser.write(message)
        # print(f"Sent to serial: {message.decode('utf-8').strip()}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass