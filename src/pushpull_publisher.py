#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Oct.10.2024
# Last update: Oct.10.2024
# Description: This script is primarily for basic data logging using UR10e robot and ATI sensors. 
# It record ATI data and robot position for 10 s. moves to initial position (position A), then it move position A to B and rotate by 45 degrees in y-axis.

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
import numpy as np
import time
import scipy
import pickle

from netft_utils.srv import *
from suction_cup.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int8
import geometry_msgs.msg
from pushpull_suctioncup_106a.msg import PushPull

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp


def main(args):

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2

  DUTYCYCLE_100 = 100
  DUTYCYCLE_30 = 30
  DUTYCYCLE_0 = 0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('pushpull_experiment')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  P_help = P_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  rospy.sleep(0.5)

  # Set the PWM Publisher  
  PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
  rospy.sleep(0.5)
  msg = PushPull()
#   msg.state = 1
#   msg.pwm = 100
#   PushPull_pub.publish(msg)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
  syncPub.publish(SYNC_RESET)
#   print("Wait for the data_logger to be enabled")
#   rospy.wait_for_service('data_logging')
#   dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
#   dataLoggerEnable(False) # reset Data Logger just in case
#   rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
#   datadir = file_help.ResultSavingDirectory
  

  # try block so that we can have a keyboard exception
  try:
    print('0 : Off')
    print('1 : Push')
    print('2 : Pull')
    

    # PWM value changes
    state = int(input("Enter a state: ") )
    if state == 0:
      msg.state = int(state)
      msg.pwm = 0
    else:
      pwm = input("Enter a PWM value: ")
    
      msg.state = int(state)
      msg.pwm = int(pwm)
    
    PushPull_pub.publish(msg)

    args.currentTime = datetime.now().strftime("%H%M%S")

    # stop data logging
    # dataLoggerEnable(False)
    # rospy.sleep(0.2)
    # P_help.stopSampling()
    # rospy.sleep(0.2)

    # # save data and clear the temporary folder
    # file_help.saveDataParams(args, appendTxt='Simple_data_log_'+'argument(int)_'+ str(args.int)+'_argument(code)_'+ str(args.currentTime))
    # file_help.clearTmpFolder()

    print("============ State Changed")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--int', type=int, help='argument for int type', default= 100)
  parser.add_argument('--str', type=str, help='argument for str type', default= "string")
  parser.add_argument('--bool', type=bool, help='argument for bool type', default= True)

  args = parser.parse_args()   
  while not rospy.is_shutdown():
    main(args)
