#!/usr/bin/env python

# Authors: Jungpyo Lee
# Description: Simple PULL→move→PUSH→move test with data saving (no haptic search)
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

    DUTYCYCLE_100 = 100
    DUTYCYCLE_30 = 30
    DUTYCYCLE_0 = 0
    PULL_STATE = 1
    PUSH_STATE = 2
    OFF_STATE  = 0

    rospy.init_node('pushpull_experiment')

    FT_help = FT_CallbackHelp()
    rospy.sleep(0.5)
    P_help = P_CallbackHelp()
    rospy.sleep(0.5)
    file_help = fileSaveHelp()
    rospy.sleep(0.5)
    rtde_help = rtdeHelp(125)
    rospy.sleep(0.5)

    targetPWM_Pub = rospy.Publisher('pwm', Int8, queue_size=1)
    rospy.sleep(0.5)
    targetPWM_Pub.publish(DUTYCYCLE_0)

    syncPub = rospy.Publisher('sync', Int8, queue_size=1)

    print("Wait for the data_logger to be enabled")
    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
    dataLoggerEnable(False)  # 데이터 로거 초기화
    rospy.sleep(1)
    file_help.clearTmpFolder()  # 임시 폴더 정리
    datadir = file_help.ResultSavingDirectory

    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
    msg = PushPull()

    # ------------------ TEST MOTION + DATA LOGGING ------------------
    try:
        input("pressure <Enter> to start to vacuum")
        targetPWM_Pub.publish(DUTYCYCLE_100)

        input("Press <Enter> to go to set bias")
        try:
            FT_help.setNowAsBias()
            rospy.sleep(0.1)
        except:
            print("set now as offset failed, but it's okay")
        P_help.startSampling()
        rospy.sleep(0.5)
        P_help.setNowAsOffset()

        # <<<< 데이터 로깅 시작 >>>>
        input("Press <Enter> to start to record data")
        dataLoggerEnable(True)
        rospy.sleep(0.2)

        # 이동 초기화
        disengagePosition = [0.38, -0.100, 0.05]
        setOrientation = tf.transformations.quaternion_from_euler(
            np.pi, 0, -np.pi/2, 'sxyz')
        disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
        rtde_help.goToPose(disEngagePose)
        rospy.sleep(0.5)

        # --- PULL 상태, 4방향 이동 ---
        msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(0.5)

        move_positions = []
        for offset in [(0.029, 0, 0), (-0.029, 0, 0), (0, 0.029, 0), (0, -0.029, 0)]:
            pos = copy.deepcopy(disengagePosition)
            pos[0] += offset[0]
            pos[1] += offset[1]
            pos[2] += offset[2]
            # pos[3] += offset[3]
            move_positions.append(pos)

        for idx, target_pos in enumerate(move_positions):
            pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
            rospy.loginfo(f"[Pull] Moving to position {idx+1}: {target_pos}")
            rtde_help.goToPose(pose_target)
            rospy.sleep(0.5)
            PushPull_pub.publish(msg)

        # --- PUSH 상태, 4방향 이동 ---
        msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(0.5)

        for idx, target_pos in enumerate(move_positions):
            pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
            rospy.loginfo(f"[Push] Moving to position {idx+1}: {target_pos}")
            rtde_help.goToPose(pose_target)
            rospy.sleep(0.5)
            PushPull_pub.publish(msg)

        # --- OFF (정지) ---
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        rospy.loginfo("=== Movement complete, suction OFF ===")

        # 예시: 실험 중 추가 PWM 명령
        targetPWM_Pub.publish(DUTYCYCLE_0)
        rospy.sleep(0.5)
        targetPWM_Pub.publish(DUTYCYCLE_30)
        rospy.sleep(0.5)
        targetPWM_Pub.publish(DUTYCYCLE_100)
        rospy.sleep(0.5)

        args.currentTime = datetime.now().strftime("%H%M%S")

        # <<<< 데이터 로깅 종료 및 파일 저장 >>>>
        dataLoggerEnable(False)
        rospy.sleep(0.2)
        P_help.stopSampling()
        targetPWM_Pub.publish(DUTYCYCLE_0)
        rospy.sleep(0.2)
        file_help.saveDataParams(args, appendTxt='Simple_data_log_movingXY_'+str(args.ch)+'_'+str(args.currentTime))
        file_help.clearTmpFolder()
        print("============ Python UR_Interface demo complete!")

    except rospy.ROSInterruptException:
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        dataLoggerEnable(False)
        P_help.stopSampling()
        return
    except KeyboardInterrupt:
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        dataLoggerEnable(False)
        P_help.stopSampling()
        return

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ch', type=int, default=4, help='number of channel')
    parser.add_argument('--tilt', type=int, default=0, help='tilted angle of the suction cup')
    parser.add_argument('--yaw', type=int, default=0, help='yaw angle of the suction cup')
    parser.add_argument('--reverse', type=bool, default=False, help='when we use reverse airflow')
    args = parser.parse_args()
    main(args)
