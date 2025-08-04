#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Nov.18.2024
# Last update: Nov.18.2024
# Description: This script is used to test 2D haptic search while recording pressure and path.

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
import string
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat


from datetime import datetime
import pandas as pd
import re
import subprocess
import numpy as np
import copy
import time
import scipy
import pickle
import shutil
from scipy.io import savemat
from scipy.spatial.transform import Rotation as sciRot


from netft_utils.srv import *
from suction_cup.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull
import geometry_msgs.msg
import tf
import cv2
from scipy import signal

from math import pi, cos, sin, floor

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp


def main(args):

  deg2rad = np.pi / 180.0
  DUTYCYCLE_100 = 100
  DUTYCYCLE_30 = 30
  DUTYCYCLE_0 = 0

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2

  PULL_STATE = 2 
  PUSH_STATE = 1
  OFF_STATE = 0 

  FT_SimulatorOn = False
  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('suction_cup')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  P_help = P_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125)
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  search_help = hapticSearch2DHelp(d_lat = 5e-3, d_yaw=1, n_ch = args.ch, p_reverse = args.reverse) # d_lat is important for the haptic search (if it is too small, the controller will fail)


  if FT_SimulatorOn:
    print("wait for FT simul")
    rospy.wait_for_service('start_sim')
    # bring the service
    netftSimCall = rospy.ServiceProxy('start_sim', StartSim)

  # Set the PushPull Publisher  
  PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
  rospy.sleep(0.5)
  msg = PushPull()
  msg.state, msg.pwm = 0, 0

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
  syncPub.publish(SYNC_RESET)

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory

  # Set TCP pose
  rtde_help.setTCPoffset([0, 0, 0.150, 0, 0, 0])
  

  # Set the disengage pose
  disengagePosition = [0.502, -0.200, 0.280]
  setOrientation = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r) #static (s) rotating (r)
  disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
  
  
  # set initial parameters
  suctionSuccessFlag = False
  # P_vac = search_help.P_vac
  timeLimit = 15
  if args.reverse:
    timeLimit = 10
  args.timeLimit = timeLimit
  pathLimit = 50e-3
  

  # try block so that we can have a keyboard exception
  try:
    input("Press <Enter> to go to disengagepose")
    rtde_help.goToPose(disEngagePose)
    
    print("Start the haptic search")
    msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
    PushPull_pub.publish(msg)
    P_help.startSampling()      
    rospy.sleep(0.5)
    P_help.setNowAsOffset()
    dataLoggerEnable(True)

    # P_vac = -7000

    print("move down to go to engagepose")
    engagePosition = copy.deepcopy(disengagePosition)
    engagePosition = [0.567, -0.0623, 0.016] # if the engage position is too loose, the controller will fail (probably because of the horizontal flow)
    engagePose = rtde_help.getPoseObj(engagePosition, setOrientation)
    rtde_help.goToPose(engagePose)

    print("Start the haptic search")
    # set initial parameters
    suctionSuccessFlag = False
    # P_vac = search_help.P_vac
    startTime = time.time()
    sg = PushPull()
    
    # begin the haptic search
    while not suctionSuccessFlag:   # while no success in grasp, run controller until success or timeout

      # msg.state = 2
      # msg.pwm = 100
      # PushPull_pub.publish(msg)

      iteration_start_time = time.time()
      
      # P arrays to calculate Transformation matrices and change the order of pressure
      P_array = P_help.four_pressure     
      #print(P_array) 
      
      # get the current yaw angle of the suction cup
      measuredCurrPose = rtde_help.getCurrentPose()
      T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)

      # calculate transformation matrices
      T_later, T_align = search_help.get_Tmats_from_controller(P_array)
      #T_later = search_help.get_Tmat_TranslateInZ()
      T_move = T_later

      # move to new pose adaptively
      measuredCurrPose = rtde_help.getCurrentPose()
      currPose = search_help.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
      rtde_help.goToPoseAdaptive(currPose)
      # Create a pose 1 cm above the current target pose
      poseAbove = currPose
      poseAbove.pose.position.z += 0.1  # Add 1 cm to the z-coordinate
      poseBelow = currPose
      poseBelow.pose.position.z -= 0.1  # Add 1 cm to the z-coordinate

      # Push

      # Move to the pose 1 cm above
      
      # while poseAbove.pose.position.z * 1000000 // 1 != rtde_help.getCurrentPose().pose.position.z * 1000000 // 1:
      #     try:
      #       print(poseAbove.pose.position.z * 1000000 // 1 , rtde_help.getCurrentPose().pose.position.z * 1000000 // 1 )
      #       rtde_help.goToPoseAdaptive(poseAbove)
      #     except KeyboardInterrupt:
      #       break



      # Move to the original target pose
      # rtde_help.goToPose(currPose)
      # Pull
      # msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
      # PushPull_pub.publish(msg)
      measuredCurrPose = rtde_help.getCurrentPose()
      currPose = search_help.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)

      # calculate current angle
      measuredCurrPose = rtde_help.getCurrentPose()
      T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)


      #=================== check attempt break conditions =================== 
      # LOOP BREAK CONDITION 1
      P_array = P_help.four_pressure

      reached_vacuum = all(np.array(P_array)<P_vac)
      # N, S, W, E
      if reached_vacuum:
        # vacuum seal formed, success!
        suctionSuccessFlag = True
        args.elapsedTime = time.time()-startTime
        print("Suction engage succeeded with controller")
        # stop at the last pose
        rtde_help.stopAtCurrPoseAdaptive()
        # keep X sec of data after alignment is complete
        rospy.sleep(1)
        break
      
      # LOOP BREAK CONDITION 2
      # if timeout, or displacement/angle passed, failed
      elif time.time()-startTime >timeLimit:
        args.timeOverFlag = time.time()-startTime >timeLimit
        args.elapsedTime = time.time()-startTime
        suctionSuccessFlag = False
        print("Suction controller failed!")

        # stop at the last pose
        rtde_help.stopAtCurrPoseAdaptive()
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        
        # keep X sec of data after alignment is complete
        rospy.sleep(0.1)
        break
      print(np.any(np.array(P_array) < P_vac))
      # elif np.any(np.array(P_array) < P_vac):
      #   direction = np.where(np.array(P_array) < P_vac)[0]
      #   print("direction: ", direction)
      #   msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
      #   PushPull_pub.publish(msg)
      #   rtde_help.goToPoseAdaptive(poseAbove)
      #   rospy.sleep(5)

    args.suction = suctionSuccessFlag
    # stop data logging
    rospy.sleep(0.2)
    dataLoggerEnable(False)
    rospy.sleep(0.2)
    P_help.stopSampling()


    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='Simple_2D_HapticSearch_' + str(args.reverse)+'_ch_'+ str(args.ch))
    file_help.clearTmpFolder()
    
    # go to disengage pose
    print("Start to go to disengage pose")
    rtde_help.goToPose(disEngagePose)
    msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
    PushPull_pub.publish(msg) 


    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
    PushPull_pub.publish(msg)
    return
  except KeyboardInterrupt:
    msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
    PushPull_pub.publish(msg)
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--ch', type=int, help='number of channel', default= 4)
  parser.add_argument('--tilt', type=int, help='tilted angle of the suction cup', default= 0)
  parser.add_argument('--yaw', type=int, help='yaw angle of the suction cup', default= 0)
  parser.add_argument('--reverse', type=bool, help='when we use reverse airflow', default= False)

  args = parser.parse_args()    
  
  main(args)
  # main(depth, rotAngleList[mode], translateZList[mode])