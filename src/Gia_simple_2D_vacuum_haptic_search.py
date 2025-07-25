#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Nov.18.2024
# Last update: 2025-07-24
# Description: Move robot in X+, X-, Y+, Y- directions with continuous PWM publishing for suction

try:
    import rospy
    import tf
    ros_enabled = True
except:
    print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
    ros_enabled = False

import numpy as np
import copy
import time

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp

from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull


def publish_pwm_continuously(pub, pwm_value, duration, rate_hz=20):
    """
    pwm_value 값을 주어진 시간 동안 지속적으로 publish합니다.
    
    Args:
        pub : ROS 퍼블리셔 객체 (pwm topic)
        pwm_value : int, 퍼블리시할 PWM 값
        duration : float, 지속시간 (초)
        rate_hz : int, 퍼블리시 주기 (Hz)
    """
    rate = rospy.Rate(rate_hz)
    end_time = time.time() + duration
    while time.time() < end_time and not rospy.is_shutdown():
        pub.publish(pwm_value)
        rate.sleep()


def main(args):
    DUTYCYCLE_100 = 100
    OFF_STATE = 0
    PULL_STATE = 2

    rospy.init_node('suction_cup_pwm_continuous')

    # 헬퍼 객체들 초기화
    FT_help = FT_CallbackHelp()
    rospy.sleep(0.5)
    P_help = P_CallbackHelp()
    rospy.sleep(0.5)
    rtde_help = rtdeHelp(125)
    rospy.sleep(0.5)
    file_help = fileSaveHelp()
    search_help = hapticSearch2DHelp(d_lat=5e-3, d_yaw=1, n_ch=args.ch, p_reverse=args.reverse)

    # 퍼블리셔 초기화
    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
    targetPWM_Pub = rospy.Publisher('pwm', Int8, queue_size=1)
    rospy.sleep(0.5)

    # Pull 상태 및 PWM 100으로 세팅
    msg = PushPull()
    msg.state = PULL_STATE
    msg.pwm = DUTYCYCLE_100
    PushPull_pub.publish(msg)

    # 시작 위치(disengage position) 및 orientation 세팅
    disengagePosition = [0.502, -0.200, 0.280]
    setOrientation = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2, 'sxyz')
    disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)

    # disengage 위치로 이동
    rospy.loginfo("Moving to disengage position...")
    rtde_help.goToPose(disEngagePose)
    rospy.sleep(1.0)

    # PWM 신호를 지속적으로 발행하면서 동작 유지 (예: 3초)
    publish_pwm_continuously(targetPWM_Pub, DUTYCYCLE_100, duration=3.0)

    # 움직일 목표 좌표 리스트 (X+, X-, Y+, Y- 방향으로 0.2m씩)
    move_positions = []
    for offset in [(0.08, 0, 0), (-0.08, 0, 0), (0, 0.08, 0), (0, -0.08, 0)]:
        pos = copy.deepcopy(disengagePosition)
        pos[0] += offset[0]
        pos[1] += offset[1]
        pos[2] += offset[2]
        move_positions.append(pos)

    # 각 위치로 순차 이동하며 PWM 지속 발행
    for idx, target_pos in enumerate(move_positions):
        pose_target = rtde_help.getPoseObj(target_pos, setOrientation)
        rospy.loginfo(f"Moving to position {idx+1}: {target_pos}")
        rtde_help.goToPose(pose_target)

        # 예상 이동시간 동안 PWM 지속 발행 (1초간)
        publish_pwm_continuously(targetPWM_Pub, DUTYCYCLE_100, duration=1.0)

        # PushPull 메시지도 계속 유지 (필요하다면 계속 퍼블리시)
        PushPull_pub.publish(msg)

    # 이동 완료 후 솔레노이드 끄기(OFF)
    rospy.loginfo("Stopping suction and PWM...")
    msg.state = OFF_STATE
    msg.pwm = 0
    PushPull_pub.publish(msg)

    targetPWM_Pub.publish(0)

    rospy.loginfo("=== Movement complete, suction OFF ===")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ch', type=int, default=4, help='number of channel')
    parser.add_argument('--tilt', type=int, default=0, help='tilted angle of the suction cup')
    parser.add_argument('--yaw', type=int, default=0, help='yaw angle of the suction cup')
    parser.add_argument('--reverse', type=bool, default=False, help='when we use reverse airflow')

    args = parser.parse_args()
    main(args)
