#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Aug.26.2024
# Last update: Aug.26.2024
# Description: This script is used to test 2D haptic search models while recording pressure and path.
#              the difference between this script and JP_2D_haptic_search_continuous.py is that this script has a hopping motion

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
import geometry_msgs.msg
import tf
import cv2
from scipy import signal

from math import pi, cos, sin, floor

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp


def pressure_order_change(P_array, ch):
  if ch == 3:
    P_array_new = [P_array[1], P_array[2], P_array[0]]
  elif ch == 4:
    P_array_new = [P_array[1], P_array[2], P_array[3], P_array[0]]
  elif ch == 6:
    P_array_new = [P_array[1], P_array[2], P_array[3], P_array[4], P_array[5], P_array[0]]
  return P_array_new

def convert_yawAngle(yaw_radian):
  yaw_angle = yaw_radian*180/pi
  yaw_angle = -yaw_angle - 90
  return yaw_angle

def get_disEngagePosition(primitives):
  if primitives == 'nozzle':
    disEngagePosition =  [-0.632, .265, 0.0151 + 5e-3]
  elif primitives == 'dumbbell':
    disEngagePosition =  [-0.665, .305, 0.0151 + 5e-3]
  elif primitives == 'test':
    disEngagePosition =  [-0.642, .290, 0.0151 + 5e-3]
  return disEngagePosition


def main(args):
  
  from helperFunction.SuctionP_callback_helper import P_CallbackHelp

  deg2rad = np.pi / 180.0
  DUTYCYCLE_100 = 100
  DUTYCYCLE_30 = 30
  DUTYCYCLE_0 = 0

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2

  FT_SimulatorOn = False
  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('suction_cup')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  P_help = P_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  rtde_help = rtde_help = rtdeHelp(125)
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  search_help = hapticSearch2DHelp(d_lat = 5e-3, d_yaw=3, n_ch = args.ch, p_reverse = args.reverse) # d_lat is important for the haptic search (if it is too small, the controller will fail)

  # Set the TCP offset and calibration matrix
  rospy.sleep(0.5)
  rtde_help.setTCPoffset([0, 0, 0.150, 0, 0, 0])
  if args.ch == 6:
    rtde_help.setTCPoffset([0, 0, 0.150 + 0.02, 0, 0, 0])
  rospy.sleep(0.2)

  if FT_SimulatorOn:
    print("wait for FT simul")
    rospy.wait_for_service('start_sim')
    # bring the service
    netftSimCall = rospy.ServiceProxy('start_sim', StartSim)

  # Set the PWM Publisher  
  targetPWM_Pub = rospy.Publisher('pwm', Int8, queue_size=1)
  rospy.sleep(0.5)
  targetPWM_Pub.publish(DUTYCYCLE_0)

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
  
  # Set the disengage pose
  disengagePosition = get_disEngagePosition(args.primitives)
  setOrientation = tf.transformations.quaternion_from_euler(pi,0,pi/2 + pi/180*args.yaw,'sxyz') #static (s) rotating (r)
  disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
  
  
  # set initial parameters
  suctionSuccessFlag = False
  controller_str = args.controller
  P_vac = search_help.P_vac
  timeLimit = 15
  args.timeLimit = timeLimit
  pathLimit = 50e-3
  

  # try block so that we can have a keyboard exception
  try:
    input("Press <Enter> to go to disengagepose")
    rtde_help.goToPose(disEngagePose)
    
    print("Start the haptic search")
    targetPWM_Pub.publish(DUTYCYCLE_100)
    P_help.startSampling()      
    rospy.sleep(1)
    P_help.setNowAsOffset()
    dataLoggerEnable(True)

    print("move down to go to engagepose")
    engagePosition = copy.deepcopy(disengagePosition)
    engagePosition[2] = disengagePosition[2] - 7e-3 # if the engage position is too loose, the controller will fail (probably because of the horizontal flow)
    engagePose = rtde_help.getPoseObj(engagePosition, setOrientation)
    rtde_help.goToPose_2Dhaptic(engagePose)

    print("Start the haptic search with hopping motion")
    # set initial parameters
    suctionSuccessFlag = False
    controller_str = args.controller
    P_vac = search_help.P_vac
    startTime = time.time()
    iteration_count = 0 # to check the frequency of the loop
    pose_diff = 0
    
    # begin the haptic search
    while not suctionSuccessFlag:   # while no success in grasp, run controller until success or timeout

      iteration_start_time = time.time()
      
      # P arrays to calculate Transformation matrices and change the order of pressure
      P_array_old = P_help.four_pressure
      P_array = pressure_order_change(P_array_old, args.ch)
      
      
      # get the current yaw angle of the suction cup
      measuredCurrPose = rtde_help.getCurrentPose()
      T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)
      yaw_angle = convert_yawAngle(search_help.get_yawRotation_from_T(T_curr))

      # calculate transformation matrices
      T_later, T_yaw, T_align = search_help.get_Tmats_from_controller(P_array, yaw_angle, controller_str)
      T_move =  T_later @ T_yaw @ T_align  # lateral --> align --> normal

      # move to new pose adaptively with hopping motion
      measuredCurrPose = rtde_help.getCurrentPose()
      currPose = search_help.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
      currPose.pose.position.z = currPose.pose.position.z + 7e-3
      rtde_help.goToPose_2Dhaptic(currPose)
      rospy.sleep(0.05)
      currPose.pose.position.z = currPose.pose.position.z - 7e-3 - pose_diff
      rtde_help.goToPose_2Dhaptic(currPose)
      rospy.sleep(0.05)
      
      # calculate current angle position
      measuredCurrPose = rtde_help.getCurrentPose()
      T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)
      args.final_yaw = convert_yawAngle(search_help.get_yawRotation_from_T(T_curr))
      pose_diff = measuredCurrPose.pose.position.z - currPose.pose.position.z # need to compensate the difference between the desired pose and the actual pose


      #=================== check attempt break conditions =================== 
      # LOOP BREAK CONDITION 1
      P_array_old = P_help.four_pressure
      P_array = pressure_order_change(P_array_old, args.ch)
      reached_vacuum = all(np.array(P_array)<P_vac)
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
        targetPWM_Pub.publish(DUTYCYCLE_0)
        
        # keep X sec of data after alignment is complete
        rospy.sleep(0.1)
        break

      # check loop frequency
      iteration_count += 1
      iteration_end_time = time.time()

       # Measure frequency every 100 iterations
      if iteration_count % 100 == 0:
          current_time = time.time()
          elapsed_time = current_time - startTime
          frequency = iteration_count / elapsed_time
          # print(f"Current control frequency after {iteration_count} iterations: {frequency} Hz")
      

    args.suction = suctionSuccessFlag
    args.iteration_count = iteration_count
    print("Press <Enter> to go stop the recording")
    # stop data logging
    rospy.sleep(0.2)
    dataLoggerEnable(False)
    rospy.sleep(0.2)
    P_help.stopSampling()
    targetPWM_Pub.publish(DUTYCYCLE_0)

    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='jp_2D_HapticSearch_hopping_' + str(args.primitives)+'_controller_'+ str(args.controller) +'_material_' + str(args.material))
    file_help.clearTmpFolder()
    
    # go to disengage pose
    print("Start to go to disengage pose")
    rtde_help.goToPose(disEngagePose)

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    targetPWM_Pub.publish(DUTYCYCLE_0)
    return
  except KeyboardInterrupt:
    targetPWM_Pub.publish(DUTYCYCLE_0)
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--primitives', type=str, help='types of primitives (nozzle, dumbbell, etc.)', default= "nozzle")
  parser.add_argument('--ch', type=int, help='number of channel', default= 4)
  parser.add_argument('--controller', type=str, help='2D haptic contollers', default= "normal")
  parser.add_argument('--material', type=int, help='Moldmax: 0, Elastic50: 1, agilus30: 2', default= 0)
  parser.add_argument('--tilt', type=int, help='tilted angle of the suction cup', default= 0)
  parser.add_argument('--yaw', type=int, help='yaw angle of the suction cup', default= 0)
  parser.add_argument('--reverse', type=bool, help='when we use reverse airflow', default= False)

  args = parser.parse_args()    
  
  main(args)
  # main(depth, rotAngleList[mode], translateZList[mode])