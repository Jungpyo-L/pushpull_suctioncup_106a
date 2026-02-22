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
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull


from helperFunction.rtde_helper import rtdeHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp




def main():


  deg2rad = np.pi / 180.0


  np.set_printoptions(precision=4)


  # controller node
  rospy.init_node('edg_experiment')


  # Setup helper functions
  rtde_help = rtdeHelp(125)
  # adaptMotionHelp: dw는 생성 시 도(deg)로 넘기며 내부에서 rad로 저장됨. 루프 내에서는 매 스텝 각도(rad)를 설정함.
  adaptHelp = adaptMotionHelp(d_lat=0.0010, dw=0.3, d_z=0.0010)

  # === 회전 P 제어 (적응형, 안전을 위해 모든 각도는 도(deg) 단위로 설정 후 rad로만 변환) ===
  Kp_rot_deg = 0.04       # [deg/압력차] P_E-P_W 1당 회전량 (도). 작을수록 부드러움
  max_rot_deg = 0.5       # 스텝당 최대 회전 각도 (도). 실험 안전용 상한
  min_rot_deg = 0.05      # 이 값 미만이면 회전 없음 (데드존, 흔들림 방지)

  # === Z(수직) P 제어: 압력 10 유지 (모자라면 내려가고, 많으면 올라감) ===
  base_d_z_m = 0.001      # 기준 Z 스텝 [m]
  max_d_z_m = 0.002       # 스텝당 최대 Z 이동 [m]. 부드럽게 하기 위한 상한
  Kp_z = 0.00015          # [m/압력차] (target - mean) 1당 Z 이동량

  P_help = P_CallbackHelp()  # Pressure sensor helper
  rospy.sleep(0.5)
 
  # === PushPull 토픽/메세지 ===
  DUTYCYCLE_100 = 100
  DUTYCYCLE_0 = 0
  PUSH_STATE = 2
  OFF_STATE = 0
  PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
  rospy.sleep(0.5)
  msg = PushPull()


  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)


  # Set the pose A
  positionA = [0.51010, 0.-0.08520, 0.03163]  # Starting position
#   positionA = [0.58678, 0.01299, 0.02846]  # Starting position

  positionA_y_end = (0.01299 - 0.08)  # Target y position (10cm from start: 0.02501 + 0.08 = 0.10501)
  orientationA = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)




  # try block so that we can have a keyboard exception
  try:


    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    rospy.sleep(1)
    print("poseA: ", rtde_help.getCurrentPose())


    # Start pressure sampling
    P_help.startSampling()
    rospy.sleep(0.5)
    P_help.setNowAsOffset()
    rospy.sleep(0.5)


    # Start push (PUSH ON)
    print("Starting push...")
    msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
    PushPull_pub.publish(msg)
    rospy.sleep(0.1)


    input("Press <Enter> to start surface following")


    # Z축 목표 압력 유지. 모자라면 내려가고, 많으면 올라감
    target_pressure = 8.0
    pressure_threshold = 5.0   # 이 값 이하는 0으로 필터
    z_tolerance = 1.0          # |평균압력 - target| < 이 값이면 Z 유지

    # ---------- 회전: P_E 경향(증가/감소 연속 횟수)으로 방향 결정 ----------
    trend_count_threshold = 3  # 이 횟수 넘으면 경향으로 판단 → 방향 전환
    prev_P_E = None
    P_E_up_count = 0    # P_E가 연속으로 커진 횟수
    P_E_down_count = 0  # P_E가 연속으로 작아진 횟수

    while 1:
        # Get pressure data
        rospy.sleep(0.05)  # Small delay to allow pressure data to update
        pressure_avg = P_help.four_pressure
       
        # Filter: raw pressure <= pressure_threshold -> 0
        pressure_filtered = [p if p > pressure_threshold else 0.0 for p in pressure_avg]
       
        # Calculate P_E, P_W, P_N, P_S
        # P_E = (1st + 2nd) / 2
        # P_W = (3rd + 4th) / 2
        # P_N = (2nd + 3rd) / 2
        # P_S = (1st + 4th) / 2
        P_E = (pressure_filtered[0] + pressure_filtered[1]) / 2.0
        P_W = (pressure_filtered[2] + pressure_filtered[3]) / 2.0
        P_N = (pressure_filtered[1] + pressure_filtered[2]) / 2.0
        P_S = (pressure_filtered[0] + pressure_filtered[3]) / 2.0
       
        # Calculate average pressure
        pressure_mean = np.mean(pressure_filtered)
        pressure_diff = abs(pressure_mean - target_pressure)
        pressure_error = target_pressure - pressure_mean   # 양수: 압력 부족(내려감), 음수: 압력 과다(올라감)

        # ---------- 회전: P_E가 커지는 경향(연속 3회↑) → 반시계, 작아지는 경향(연속 3회↓) → 시계 ----------
        if prev_P_E is not None:
            if P_E > prev_P_E:
                P_E_up_count += 1
                P_E_down_count = 0
            elif P_E < prev_P_E:
                P_E_down_count += 1
                P_E_up_count = 0
            else:
                P_E_up_count = 0
                P_E_down_count = 0
        prev_P_E = P_E

        if P_E_up_count >= trend_count_threshold:
            align_direction = 1   # 반시계 (CCW)
            rotation_angle_deg = max_rot_deg
        elif P_E_down_count >= trend_count_threshold:
            align_direction = -1  # 시계 (CW)
            rotation_angle_deg = max_rot_deg
        else:
            align_direction = 0
            rotation_angle_deg = 0.0
        rotation_angle_rad = abs(rotation_angle_deg) * (np.pi / 180.0)

        # Lateral: 고정 (오른쪽으로)
        T_later = adaptHelp.get_Tmat_TranlateInY(direction=-1)

        # Align: 경향에 따른 회전 (각도는 도→rad 한 번만 변환)
        if align_direction != 0:
            adaptHelp.dw = rotation_angle_rad
            T_align = adaptHelp.get_Tmats_RotationAtX(direction=align_direction)
        else:
            T_align = np.eye(4)

        # ---------- Z: 목표 압력 유지 P 제어 (모자라면 내려가고, 많으면 올라감) ----------
        if pressure_diff < z_tolerance:
            step_z_m = 0.0
            z_direction = 0
        else:
            step_z_m = Kp_z * pressure_error
            step_z_m = np.clip(step_z_m, -max_d_z_m, max_d_z_m)
            z_direction = int(np.sign(step_z_m))
        if z_direction != 0:
            adaptHelp.d_z_normal = abs(step_z_m)
            T_normalMove = adaptHelp.get_Tmat_TranlateInZ(direction=z_direction)
            adaptHelp.d_z_normal = base_d_z_m   # 다음 루프를 위해 기본값 복원
        else:
            T_normalMove = np.eye(4)
       
        # Combine transformations: lateral --> align --> normal
        T_move = T_later @ T_align @ T_normalMove
       
        # Move to new pose adaptively
        measuredCurrPose = rtde_help.getCurrentPose()
        deltaPose = adaptHelp.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        rtde_help.goToPoseAdaptive(deltaPose)
       
        # Get current y position to check if we've reached the target
        currentPose = rtde_help.getCurrentPose()
        current_y = currentPose.pose.position.y
       
        # Debug print (P_E 경향: up/down count, 회전은 도 단위)
        print(f"Y: {current_y:.5f} | P_E: {P_E:.2f}, P_W: {P_W:.2f} | up#{P_E_up_count} down#{P_E_down_count} | Rot: {rotation_angle_deg:.4f}deg dir {align_direction} | Mean: {pressure_mean:.2f}, err: {pressure_error:.2f} | Z: {z_direction} step {step_z_m*1000:.3f}mm")


        if rospy.is_shutdown():
            # Stop push before returning
            msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            P_help.stopSampling()
            return
       
        # Check if we've reached the target y position (10cm from start)
        # Start: 0.02501, Target: 0.10501 (10cm = 0.08m away)
        # Check distance traveled (works for both +y and -y directions)
        y_distance_traveled = abs(current_y - positionA[1])  # Distance from start
        target_distance = 0.10  # 10cm = 0.10m
       
        if y_distance_traveled >= target_distance:
            print(f"Reached target distance! Traveled: {y_distance_traveled:.5f}m (target: {target_distance:.5f}m)")
            print(f"Current y: {current_y:.5f}, Start y: {positionA[1]:.5f}, Target y: {positionA_y_end:.5f}")
            break
   
    # Stop push (PUSH OFF)
    print("Stopping push...")
    msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
    PushPull_pub.publish(msg)
    rospy.sleep(0.1)
   
    # Stop pressure sampling
    P_help.stopSampling()


   
   
   


    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  




if __name__ == '__main__':
  main()


