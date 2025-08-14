#!/usr/bin/env python
## ESP32-S3 feather pressure sensor reading and publish
import rospy
import numpy as np
import os, sys
from pushpull_suctioncup_106a.msg import SensorPacket
from pushpull_suctioncup_106a.msg import cmdPacket

# serial communication
import serial
from serial import Serial
import struct
import argparse

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

def main(args):
    global currState
    global CMD_in

    rospy.init_node('ESP32_Pressure')
    rospy.loginfo("Num of channels (args.ch): %d", args.ch)

    # Sensor reading is published to topic 'SensorPacket'
    pub = rospy.Publisher('SensorPacket', SensorPacket, queue_size=10)
    rospy.Subscriber("cmdPacket", cmdPacket, callback)

    msg = SensorPacket()
    msg.ch = args.ch                          # ch 
    msg.data = [0.0] * args.ch                 

    # Serial communication setup
    ser = serial.Serial("/dev/ttyPressure", baudrate=115200, timeout=1, write_timeout=1)
    ser.flushInput()

    while not rospy.is_shutdown():
        try:
            CMD_in = START_CMD
            if currState == IDLE and CMD_in == START_CMD:
                rospy.loginfo("Streaming start")
                CMD_in = NO_CMD
                ser.write(struct.pack('<B', ord("i")))
                rospy.sleep(0.01)
                ser.write(struct.pack('<B', ord("s")))
                rospy.sleep(0.01)

                while not CMD_in == IDLE_CMD and not rospy.is_shutdown():
                    ser_bytes = ser.readline().decode("utf-8").strip()
                    split_data = ser_bytes.split(' ')

                    if len(split_data) < args.ch:
                        rospy.logwarn("Received fewer data points than expected. Got %d, expected %d",
                                      len(split_data), args.ch)

                        continue

                    # publishing sensor data
                    for i in range(args.ch):
                        msg.data[i] = float(split_data[i])

                    msg.header.stamp = rospy.Time.now()
                    pub.publish(msg)
                    print(msg)

                # STOP signal
                ser.write(struct.pack('<B', ord("i")))
                ser.flushInput()
                rospy.sleep(0.01)            
                CMD_in = NO_CMD
                currState = IDLE
        
        except Exception as e:
            rospy.logerr("SensorComError: " + str(e))
            pass

    rospy.loginfo("ESP32 Sensor Reading Done")

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--ch', type=int, help='number of channel', default=4)
        args = parser.parse_args()    
        main(args)
    except rospy.ROSInterruptException: 
        pass
