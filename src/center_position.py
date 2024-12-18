#!/usr/bin/env python

try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

from calendar import month_abbr
import os, sys
import numpy as np
import copy
from pushpull_suctioncup_106a.msg import PushPull
from netft_utils.srv import *
from edg_ur10.srv import *

from helperFunction.rtde_helper import rtdeHelp

def main():

  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('center_position')

  # Setup helper functions
  rtde_help = rtdeHelp(125)
  rospy.sleep(0.5)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)
  # PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
  # rospy.sleep(0.5)
  # msg = PushPull()
  # msg.pwm = 0
  # msg.state = 0
  # PushPull_pub.publish(msg)
  # Set the pose A
  # positionA = [0.402, 0.2, 0.280]
  
  # orientationA = tf.transformations.quaternion_from_euler(np.pi,0,np.pi,'sxyz') #static (s) rotating (r)
  # positionA = [0.5665, -0.0449, 0.28]
  # poseA = rtde_help.getPoseObj(positionA, orientationA)

  # # try block so that we can have a keyboard exception
  # try:
  #   input("Press <Enter> to go to pose A")
  #   rtde_help.goToPose(poseA)
  try:
    #print(rtde_help.getCurrentPose())
    # orientationA = [0.01082, -0.72230, -0.69148, 0.00368]
    # positionA = [0.58, 0.106, 0.29]
    # poseA = rtde_help.getPoseObj(positionA, orientationA)
    # input("Press <Enter> to go to detection pose")
    # rtde_help.goToPose(poseA)

    positionB = np.array([0.4763518817033466, 0.013583290787995318, 0.03])

    #need to be adjusted depends on the camera setting
    constant_diff = np.array([-0.0507, -0.0, 0.0])

    correct_pososition = positionB - constant_diff
    orientationB = tf.transformations.quaternion_from_euler(np.pi,0,np.pi,'sxyz') #static (s) rotating (r)
    poseB = rtde_help.getPoseObj(correct_pososition, orientationB)
    input("Press <Enter> to go to the center")
    rtde_help.goToPose(poseB)

    print("Final center position: ", correct_pososition)


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  main()
