#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Oct.08.2024
# Last update: Oct.10.2024
# Description: This script is primarily for basic robot control using UR10e robot. 
# It first moves to initial position (position A), then it move position A (poseA) to B (poseB) and rotate by 45 degrees in y-axis (poseC).

# imports
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
import time

from netft_utils.srv import *
from edg_ur10.srv import *

from helperFunction.rtde_helper import rtdeHelp
from helperFunction.adaptiveMotion import adaptMotionHelp


def main():

  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('edg_experiment')

  # Setup helper functions
  rtde_help = rtdeHelp(125)
  adaptHelp = adaptMotionHelp(d_lat=0.005, dw=0.5, d_z=0.0015)
  rospy.sleep(0.5)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)

  # Set the pose A
  positionA = [0.45465, 0.02501, 0.23657] 
  orientationA = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)


  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    rospy.sleep(1)
    print("poseA: ", rtde_help.getCurrentPose())

    input("Press <Enter> to go to target pose")

    StartTime = time.time()
    timeLimit = 3
    while 1:
        # calculate transformation matrices
        T_later = adaptHelp.get_Tmat_TranlateInY(direction=-1)
        T_align = adaptHelp.get_Tmats_RotationAtX(direction=-1)
        T_normalMove = adaptHelp.get_Tmat_TranlateInZ(direction=1)

        T_move =  T_later @ T_align @ T_normalMove # lateral --> align --> normal
        # move to new pose adaptively
        measuredCurrPose = rtde_help.getCurrentPose()
        deltaPose = adaptHelp.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        rtde_help.goToPoseAdaptive(deltaPose)

        if rospy.is_shutdown():
            return
        
        if time.time()-StartTime >timeLimit:
          break

    
    StartTime = time.time()

    while 1:
        # calculate transformation matrices
        T_later = adaptHelp.get_Tmat_TranlateInY(direction=-1)
        T_align = adaptHelp.get_Tmats_RotationAtX(direction=1)
        T_normalMove = adaptHelp.get_Tmat_TranlateInZ(direction=1)

        T_move =  T_later @ T_align @ T_normalMove # lateral --> align --> normal
        # move to new pose adaptively
        measuredCurrPose = rtde_help.getCurrentPose()
        deltaPose = adaptHelp.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        rtde_help.goToPoseAdaptive(deltaPose)

        if rospy.is_shutdown():
            return
        
        if time.time()-StartTime >timeLimit:
          break

    
    
    

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  main()
