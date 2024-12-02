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
import string
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat
import numpy as np
from helperFunction.rtde_helper import rtdeHelp


def shoot_basketball(rtde_help):
  deg2rad = np.pi / 180.0
  DUTYCYCLE_100 = 100
  DUTYCYCLE_30 = 30
  DUTYCYCLE_0 = 0
  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2
  
  FT_SimulatorOn = False
  np.set_printoptions(precision=4)
  positionA = [0.502, -0.200, 0.280]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)
  try:
    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    print("============ Return Home ============")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  
