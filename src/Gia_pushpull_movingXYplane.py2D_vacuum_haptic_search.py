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
    DUTYCYCLE_0 = 0
    PULL_STATE = 1   # 변경됨: PULL은 1
    PUSH_STATE = 2   # 변경됨: PUSH는 2
    OFF_STATE  = 0

    rospy.init_node('suction_cup_simple_with_save')

    FT_help = FT_CallbackHelp()
    rospy.sleep(0.5)
    P_help = P_CallbackHelp()
    rospy.sleep(0.5)
    rtde_help = rtdeHelp(125)
    rospy.sleep(0.5)

    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
    rospy.sleep(0.5)

    msg = PushPull()

    # disengagePosition = [0.502, -0.200, 0.280]
    disengagePosition = [0.38, -0.100, 0.05] #################
    setOrientation = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi/2, 'sxyz')
    disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
    rtde_help.goToPose(disEngagePose)
    rospy.sleep(0.5)

    # 데이터 기록을 위한 리스트 초기화
    movement_log = []

    try:
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
            move_positions.append(pos)

        for idx, target_pos in enumerate(move_positions):
            pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
            t1 = datetime.now()
            rospy.loginfo(f"[Pull] Moving to position {idx+1}: {target_pos}")
            rtde_help.goToPose(pose_target)
            time.sleep(0.5)
            PushPull_pub.publish(msg)
            t2 = datetime.now()
            movement_log.append({
                'Time1': t1.strftime("%Y-%m-%d %H:%M:%S.%f"),
                'State': PULL_STATE,
                'Time2': t2.strftime("%Y-%m-%d %H:%M:%S.%f"),
                'PWM': DUTYCYCLE_100,
                'X': target_pos[0],
                'Y': target_pos[1],
                'Z': target_pos[2]
            })

        # --- PUSH 상태, 4방향 이동 ---
        msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(0.5)

        for idx, target_pos in enumerate(move_positions):
            pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
            t1 = datetime.now()
            rospy.loginfo(f"[Push] Moving to position {idx+1}: {target_pos}")
            rtde_help.goToPose(pose_target)
            time.sleep(0.5)
            PushPull_pub.publish(msg)
            t2 = datetime.now()
            movement_log.append({
                'Time1': t1.strftime("%Y-%m-%d %H:%M:%S.%f"),
                'State': PUSH_STATE,
                'Time2': t2.strftime("%Y-%m-%d %H:%M:%S.%f"),
                'PWM': DUTYCYCLE_100,
                'X': target_pos[0],
                'Y': target_pos[1],
                'Z': target_pos[2]
            })

        # --- OFF (정지) ---
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        rospy.loginfo("=== Movement complete, suction OFF ===")
        t1 = datetime.now()
        t2 = datetime.now()
        movement_log.append({
            'Time1': t1.strftime("%Y-%m-%d %H:%M:%S.%f"),
            'State': OFF_STATE,
            'Time2': t2.strftime("%Y-%m-%d %H:%M:%S.%f"),
            'PWM': DUTYCYCLE_0,
            'X': '',
            'Y': '',
            'Z': ''
        })

        # --- 데이터 CSV로 저장 ---
        df = pd.DataFrame(movement_log)
        # df.to_csv("Gia_pushpull_movingXYplane.csv", index=False)
        df.to_csv("/home/edg/catkin_ws/src/pushpull_suctioncup_106a/src/Gia_data/0.100Z_Gia_pushpull_movingXYplane.csv", index=False)
        #################
        print("Saved to Gia_pushpull_movingXYplane.csv in current directory.")

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
    parser.add_argument('--ch', type=int, default=4, help='number of channel')
    parser.add_argument('--tilt', type=int, default=0, help='tilted angle of the suction cup')
    parser.add_argument('--yaw', type=int, default=0, help='yaw angle of the suction cup')
    parser.add_argument('--reverse', type=bool, default=False, help='when we use reverse airflow')
    args = parser.parse_args()
    main(args)



    
# def main(args):
#     DUTYCYCLE_100 = 100
#     DUTYCYCLE_0 = 0
#     PULL_STATE = 2
#     PUSH_STATE = 1
#     OFF_STATE = 0

#     rospy.init_node('suction_cup_simple_with_save')

#     # 헬퍼 객체 초기화
#     FT_help = FT_CallbackHelp()
#     rospy.sleep(0.5)
#     P_help = P_CallbackHelp()
#     rospy.sleep(0.5)
#     rtde_help = rtdeHelp(125)
#     rospy.sleep(0.5)
#     file_help = fileSaveHelp()

#     PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
#     rospy.sleep(0.5)

#     # 데이터 로깅 서비스 대기 및 핸들러
#     rospy.wait_for_service('data_logging')
#     dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)

#     msg = PushPull()

#     # 시작 위치 및 방향
#     disengagePosition = [0.502, -0.200, 0.280]
#     setOrientation = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi/2, 'sxyz')
#     disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)

#     # 분리 위치로 이동
#     rospy.loginfo("Moving to disengage position...")
#     rtde_help.goToPose(disEngagePose)
#     rospy.sleep(0.5)

#     try:
#         # 데이터 로깅 초기화 및 시작
#         dataLoggerEnable(False)  # 초기화(비활성화)
#         rospy.sleep(0.5)
#         file_help.clearTmpFolder()  # 임시 폴더 정리
#         dataLoggerEnable(True)   # 로깅 시작

#         # 센서 샘플링 시작
#         P_help.startSampling()
#         rospy.sleep(0.5)
#         # 필요시 오프셋 설정 등 추가 가능

#         # --- PULL 상태, 4방향 이동 ---
#         msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
#         PushPull_pub.publish(msg)
#         rospy.sleep(0.5)

#         move_positions = []
#         for offset in [(0.08, 0, 0), (-0.08, 0, 0), (0, 0.08, 0), (0, -0.08, 0)]:
#             pos = copy.deepcopy(disengagePosition)
#             pos[0] += offset[0]
#             pos[1] += offset[1]
#             pos[2] += offset[2]
#             move_positions.append(pos)

#         for idx, target_pos in enumerate(move_positions):
#             pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
#             rospy.loginfo(f"[Pull] Moving to position {idx+1}: {target_pos}")
#             rtde_help.goToPose(pose_target)
#             rospy.sleep(0.5)
#             PushPull_pub.publish(msg)

#         # --- PUSH 상태, 4방향 이동 ---
#         msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
#         PushPull_pub.publish(msg)
#         rospy.sleep(0.5)

#         for idx, target_pos in enumerate(move_positions):
#             pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
#             rospy.loginfo(f"[Push] Moving to position {idx+1}: {target_pos}")
#             rtde_help.goToPose(pose_target)
#             rospy.sleep(0.5)
#             PushPull_pub.publish(msg)

#         # --- OFF (정지) ---
#         msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
#         PushPull_pub.publish(msg)
#         rospy.loginfo("=== Movement complete, suction OFF ===")

#         # 센서 샘플링 종료
#         P_help.stopSampling()

#         # 데이터 로깅 종료
#         rospy.sleep(0.2)
#         dataLoggerEnable(False)
#         rospy.sleep(0.2)

#         # 데이터 저장 및 임시 폴더 정리
#         file_help.saveDataParams(args, appendTxt='Gia_pushpull_movingXYplane' + str(args.ch))
#         file_help.clearTmpFolder()

#     except rospy.ROSInterruptException:
#         msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
#         PushPull_pub.publish(msg)
#         dataLoggerEnable(False)
#         P_help.stopSampling()
#         return
#     except KeyboardInterrupt:
#         msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
#         PushPull_pub.publish(msg)
#         dataLoggerEnable(False)
#         P_help.stopSampling()
#         return

# if __name__ == '__main__':
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--ch', type=int, default=4, help='number of channel')
#     parser.add_argument('--tilt', type=int, default=0, help='tilted angle of the suction cup')
#     parser.add_argument('--yaw', type=int, default=0, help='yaw angle of the suction cup')
#     parser.add_argument('--reverse', type=bool, default=False, help='when we use reverse airflow')

#     args = parser.parse_args()
#     main(args)

# #!/usr/bin/env python

# # Authors: Jungpyo Lee
# # Create: Nov.18.2024
# # Last update: 2025-07-24
# # Description: Move robot in X+, X-, Y+, Y- directions with continuous PWM publishing for suction

# try:
#     import rospy
#     import tf
#     ros_enabled = True
# except:
#     print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
#     ros_enabled = False

# import numpy as np
# import copy
# import time

# from helperFunction.FT_callback_helper import FT_CallbackHelp
# from helperFunction.fileSaveHelper import fileSaveHelp
# from helperFunction.rtde_helper import rtdeHelp
# from helperFunction.hapticSearch2D import hapticSearch2DHelp
# from helperFunction.SuctionP_callback_helper import P_CallbackHelp

# from std_msgs.msg import Int8
# from pushpull_suctioncup_106a.msg import PushPull


# def publish_pwm_continuously(pub, pwm_value, duration, rate_hz=20):
#     """
#     pwm_value 값을 주어진 시간 동안 지속적으로 publish합니다.
    
#     Args:
#         pub : ROS 퍼블리셔 객체 (pwm topic)
#         pwm_value : int, 퍼블리시할 PWM 값
#         duration : float, 지속시간 (초)
#         rate_hz : int, 퍼블리시 주기 (Hz)
#     """
#     rate = rospy.Rate(rate_hz)
#     end_time = time.time() + duration
#     while time.time() < end_time and not rospy.is_shutdown():
#         pub.publish(pwm_value)
#         rate.sleep()


# def main(args):
#     DUTYCYCLE_100 = 100
#     OFF_STATE = 0
#     PULL_STATE = 2

#     rospy.init_node('suction_cup_pwm_continuous')

#     # 헬퍼 객체들 초기화
#     FT_help = FT_CallbackHelp()
#     rospy.sleep(0.5)
#     P_help = P_CallbackHelp()
#     rospy.sleep(0.5)
#     rtde_help = rtdeHelp(125)
#     rospy.sleep(0.5)
#     file_help = fileSaveHelp()
#     search_help = hapticSearch2DHelp(d_lat=5e-3, d_yaw=1, n_ch=args.ch, p_reverse=args.reverse)

#     # 퍼블리셔 초기화
#     PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
#     targetPWM_Pub = rospy.Publisher('pwm', Int8, queue_size=1)
#     rospy.sleep(0.5)

#     # Pull 상태 및 PWM 100으로 세팅
#     msg = PushPull()
#     msg.state = PULL_STATE
#     msg.pwm = DUTYCYCLE_100
#     PushPull_pub.publish(msg)

#     # 시작 위치(disengage position) 및 orientation 세팅
#     disengagePosition = [0.502, -0.200, 0.280]
#     setOrientation = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2, 'sxyz')
#     disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)

#     # disengage 위치로 이동
#     rospy.loginfo("Moving to disengage position...")
#     rtde_help.goToPose(disEngagePose)
#     rospy.sleep(1.0)

#     # PWM 신호를 지속적으로 발행하면서 동작 유지 (예: 3초)
#     publish_pwm_continuously(targetPWM_Pub, DUTYCYCLE_100, duration=3.0)

#     # 움직일 목표 좌표 리스트 (X+, X-, Y+, Y- 방향으로 0.2m씩)
#     move_positions = []
#     for offset in [(0.08, 0, 0), (-0.08, 0, 0), (0, 0.08, 0), (0, -0.08, 0)]:
#         pos = copy.deepcopy(disengagePosition)
#         pos[0] += offset[0]
#         pos[1] += offset[1]
#         pos[2] += offset[2]
#         move_positions.append(pos)

#     # 각 위치로 순차 이동하며 PWM 지속 발행
#     for idx, target_pos in enumerate(move_positions):
#         pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
#         rospy.loginfo(f"Moving to position {idx+1}: {target_pos}")
#         rtde_help.goToPose(pose_target)

#         # 예상 이동시간 동안 PWM 지속 발행 (1초간)
#         publish_pwm_continuously(targetPWM_Pub, DUTYCYCLE_100, duration=1.0)

#         # PushPull 메시지도 계속 유지 (필요하다면 계속 퍼블리시)
#         PushPull_pub.publish(msg)

#     # 이동 완료 후 솔레노이드 끄기(OFF)
#     rospy.loginfo("Stopping suction and PWM...")
#     msg.state = OFF_STATE
#     msg.pwm = 0
#     PushPull_pub.publish(msg)

#     targetPWM_Pub.publish(0)

#     rospy.loginfo("=== Movement complete, suction OFF ===")


# if __name__ == "__main__":
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--ch', type=int, default=4, help='number of channel')
#     parser.add_argument('--tilt', type=int, default=0, help='tilted angle of the suction cup')
#     parser.add_argument('--yaw', type=int, default=0, help='yaw angle of the suction cup')
#     parser.add_argument('--reverse', type=bool, default=False, help='when we use reverse airflow')

#     args = parser.parse_args()
#     main(args)
