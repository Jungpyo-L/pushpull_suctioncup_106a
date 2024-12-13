#!/usr/bin/env python

## ESP32-S3 feather pressure sensor reading and publish import rospy

import rospy
import numpy as np
import os, sys
from pushpull_suctioncup_106a.msg import SensorPacket
from pushpull_suctioncup_106a.msg import cmdPacket

# serial communication
import serial
from serial import Serial
import struct


IDLE = 0
STREAMING = 1

NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3

currState = IDLE
CMD_in = NO_CMD


def callback(data):
    global CMD_in        
    CMD_in = data.cmdInput


def main():
    global currState
    global CMD_in

    rospy.init_node('ESP32_Pressure')
    print("cuurent state: ", currState)
    print("CMD_in: ", CMD_in)
    #Sensor reading is published to topic 'SensorPacket'
    pub = rospy.Publisher('SensorPacket', SensorPacket, queue_size=10)
    rospy.Subscriber("cmdPacket",cmdPacket, callback)

    print("cuurent state: ", currState)
    print("CMD_in: ", CMD_in)
    msg = SensorPacket()
    msg.data = [0.0, 0.0, 0.0, 0.0] 

    # ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1, write_timeout=1)
    ser = serial.Serial("/dev/ttyPressure", baudrate=115200, timeout=1, write_timeout=1)
    ser.flushInput()
 
    while not rospy.is_shutdown():
        try:
            CMD_in = START_CMD
            if currState == IDLE and CMD_in == START_CMD:
                print("true")
                CMD_in = NO_CMD
                ser.write(struct.pack('<B', ord("i")))
                rospy.sleep(0.01)
                ser.write(struct.pack('<B', ord("s")))
                rospy.sleep(0.01)
                while not CMD_in == IDLE_CMD and not rospy.is_shutdown():
                    
                    ser_bytes = ser.readline().decode("utf-8")
                    # print("ser_bytes: ", ser_bytes)
                    split_data = ser_bytes.split(' ')
                    # print("sizeof_split_data: ", len(split_data))
                    first_val = split_data[0]
                    # print("first_vale: ", first_val)
                    second_val = split_data[1]
                    third_val = split_data[2]
                    fourth_val = split_data[3]
                    msg.data[0] = float(first_val)
                    msg.data[1] = float(second_val)
                    msg.data[2] = float(third_val)
                    msg.data[3] = float(fourth_val)
                    print(msg)
                    msg.header.stamp = rospy.Time.now()
                    print("test")
                    pub.publish(msg)

                # ser.write("i" + "\r\n")
                ser.write(struct.pack('<B', ord("i")))
                ser.flushInput()
                rospy.sleep(0.01)            
                CMD_in = NO_CMD
                currState = IDLE
        
        except Exception as e:
            print("SensorComError: " + str(e))
            pass
    print("ESP32 Sensor Reading Done")

if __name__ == '__main__':
    try:
        print("Started!")
        main()

    except rospy.ROSInterruptException: 
        print("oops")
        pass
