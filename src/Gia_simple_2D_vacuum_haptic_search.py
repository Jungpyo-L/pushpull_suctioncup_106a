#!/usr/bin/env python

import rospy
import tf
import numpy as np
import time
import copy
from math import pi
from datetime import datetime

from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat
from netft_utils.srv import *
from suction_cup.srv import *
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp


def main(args):
    DUTYCYCLE_100 = 100
    DUTYCYCLE_0 = 0

    SYNC_RESET = 0
    PULL_STATE = 1
    PUSH_STATE = 2
    OFF_STATE = 0

    rospy.init_node('suction_cup')

    # Helper 초기화
    FT_help = FT_CallbackHelp()
    rospy.sleep(0.5)
    P_help = P_CallbackHelp()
    rospy.sleep(0.5)
    rtde_help = rtdeHelp(125)
    rospy.sleep(0.5)
    file_help = fileSaveHelp()

    search_help = hapticSearch2DHelp(d_lat=5e-3, d_yaw=1, n_ch=args.ch, p_reverse=args.reverse)

    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
    rospy.sleep(0.5)
    msg = PushPull()

    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)

    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
    dataLoggerEnable(False)
    rospy.sleep(1)
    file_help.clearTmpFolder()
    datadir = file_help.ResultSavingDirectory

    # ******** disengage / engage pose  ********
    disengagePosition = [0.38, -0.100, 0.280]
    setOrientation = tf.transformations.quaternion_from_euler(pi, 0, -pi/2, 'sxyz')
    disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)

    engagePosition = [0.40, -0.100, 0.280]
    engagePose = rtde_help.getPoseObj(engagePosition, setOrientation)
    # **********************************************

    timeLimit = 15 if not args.reverse else 10
    args.timeLimit = timeLimit

    suctionSuccessFlag = False

    try:
        input("Press <Enter> to go to disengagepose")
        rtde_help.goToPose(disEngagePose)

        print("Start the haptic search")
        msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
        PushPull_pub.publish(msg)
        P_help.startSampling()
        rospy.sleep(0.5)
        P_help.setNowAsOffset()
        dataLoggerEnable(True)

        rtde_help.goToPose(engagePose)

        startTime = time.time()

        while not suctionSuccessFlag:
            P_array = P_help.four_pressure
            print("Current pressure readings: ", P_array) ###THIS IS THE PROBLEM 
            measuredCurrPose = rtde_help.getCurrentPose()
            T_later, T_align = search_help.get_Tmats_from_controller(P_array)
            T_move = T_later

            currPose = search_help.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
            rtde_help.goToPoseAdaptive(currPose)

            P_vac = search_help.P_vac
            reached_vacuum = all(np.array(P_array) < P_vac)

            if reached_vacuum:
                suctionSuccessFlag = True
                args.elapsedTime = time.time() - startTime
                print("Suction engage succeeded with controller")
                rtde_help.stopAtCurrPoseAdaptive()
                rospy.sleep(1)
                break
            elif time.time() - startTime > timeLimit:
                args.elapsedTime = time.time() - startTime
                print("Suction controller failed!")
                rtde_help.stopAtCurrPoseAdaptive()
                msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
                PushPull_pub.publish(msg)
                rospy.sleep(0.1)
                break

        args.suction = suctionSuccessFlag
        dataLoggerEnable(False)
        P_help.stopSampling()

        file_help.saveDataParams(
            args, appendTxt='Simple_2D_HapticSearch_' + str(args.reverse) + '_ch_' + str(args.ch)
        )
        file_help.clearTmpFolder()

        print("Start to go to disengage pose")
        rtde_help.goToPose(disEngagePose)
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)

        print("============ Python UR_Interface demo complete!")

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ch', type=int, default=4)
    parser.add_argument('--tilt', type=int, default=0)
    parser.add_argument('--yaw', type=int, default=0)
    parser.add_argument('--reverse', type=bool, default=True)
    args = parser.parse_args()
    main(args)
