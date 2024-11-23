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

from netft_utils.srv import *
from edg_ur10.srv import *

from helperFunction.rtde_helper import rtdeHelp

def main():

  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('pushpull_experiment')

  # Setup helper functions
  rtde_help = rtdeHelp(125)
  rospy.sleep(0.5)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)
 
  # Set the pose A
  positionA = [0.502, -0.200, 0.280]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)

  # try block so that we can have a keyboard exception
  try:
    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    print("============ Return Home ============")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  main()
