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
from pushpull_suctioncup_106a.msg import PushPull

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
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from shoot_basketball import shoot_basketball

def main(args):

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
  rospy.init_node('pickup_basketball')

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

  # Set the PWM Publisher  
  PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=1)
  rospy.sleep(0.5)

  msg = PushPull()
  msg.state = 0
  msg.pwm = DUTYCYCLE_0
  PushPull_pub.publish(msg)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
  syncPub.publish(SYNC_RESET)
  try:
      print("Wait for the data_logger to be enabled")
      rospy.wait_for_service('data_logging')
      dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
      dataLoggerEnable(False) # reset Data Logger just in case
      rospy.sleep(1)
      file_help.clearTmpFolder()        # clear the temporary folder
      datadir = file_help.ResultSavingDirectory
      

      # Set the disengage pose
      disengagePosition = [0.502, .2, 0.280]
      setOrientation = tf.transformations.quaternion_from_euler(np.pi,0,np.pi,'sxyz') #static (s) rotating (r) #static (s) rotating (r)
      disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
      
      
      # set initial parameters
      suctionSuccessFlag = False
      P_vac = search_help.P_vac
      timeLimit = 10
      if args.reverse:
        timeLimit = 10
      args.timeLimit = timeLimit
      pathLimit = 50e-3
    

    # try block so that we can have a keyboard exception
      input("Press <Enter> to go to disengagepose")
      rtde_help.goToPose(disEngagePose)
      
      print("Start the haptic search")
      msg.pwm = DUTYCYCLE_100
      msg.state = 2
      PushPull_pub.publish(msg)
      P_help.startSampling()      
      rospy.sleep(0.5)
      P_help.setNowAsOffset()
      dataLoggerEnable(True)

      print("move down to go to engagepose")
      engagePosition = copy.deepcopy(disengagePosition)
      engagePosition[2] = disengagePosition[2] - 0.05 # if the engage position is too loose, the controller will fail (probably because of the horizontal flow)
      engagePose = rtde_help.getPoseObj(engagePosition, setOrientation)
      rtde_help.goToPose(engagePose)

      print("Start the haptic search")
      # set initial parameters
      suctionSuccessFlag = False
      P_vac = search_help.P_vac
      startTime = time.time()
      
      # begin the haptic search
      while not suctionSuccessFlag:   # while no success in grasp, run controller until success or timeout

        iteration_start_time = time.time()
        
        # P arrays to calculate Transformation matrices and change the order of pressure
        P_array = P_help.four_pressure      
        
        # get the current yaw angle of the suction cup
        measuredCurrPose = rtde_help.getCurrentPose()
        T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)

        # calculate transformation matrices
        T_later, T_align = search_help.get_Tmats_from_controller(P_array)
        # T_later = search_help.get_Tmat_TranslateInX()
        T_move = T_later

        # move to new pose adaptively
        measuredCurrPose = rtde_help.getCurrentPose()
        currPose = search_help.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        rtde_help.goToPoseAdaptive(currPose)
        
        # calculate current angle
        measuredCurrPose = rtde_help.getCurrentPose()
        T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)


        #=================== check attempt break conditions =================== 
        # LOOP BREAK CONDITION 1
        P_array = P_help.four_pressure

        reached_vacuum = all(np.array(P_array)<P_vac)
        if reached_vacuum:
          # vacuum seal formed, success!
          suctionSuccessFlag = True
          args.elapsedTime = time.time() - startTime
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
          msg.pwm = DUTYCYCLE_0
          msg.state = 0
          PushPull_pub.publish(msg)
          
          # keep X sec of data after alignment is complete
          rospy.sleep(0.1)
          break

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
      if(args.suction):
        shoot_basketball(rtde_help, PushPull_pub)
      rospy.sleep(0.5)
      # shooting_orientation = tf.transformations.quaternion_from_euler(np.pi - 60*deg2rad, 0, np.pi/4,'sxyz')
      # shooting_pose = rtde_help.getPoseObj(disengagePosition, [shooting_orientation[0], shooting_orientation[1], shooting_orientation[2], shooting_orientation[3]])
      # rtde_help.goToPoseAdaptive(shooting_pose)

      # rtde_help.goToPose(shooting_pose)
      # msg.pwm = DUTYCYCLE_0
      # msg.state = 0
      # PushPull_pub.publish(msg)

      print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    msg.pwm = DUTYCYCLE_0
    msg.state = 0
    PushPull_pub.publish(msg)   
    return
  
  except KeyboardInterrupt:
    msg.pwm = DUTYCYCLE_0
    msg.state = 0
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