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
  adaptHelp = adaptMotionHelp(d_lat=0.0015, dw=0.55, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down
# adaptHelp = adaptMotionHelp(d_lat=0.005, dw=0.5, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down

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
  positionA = [0.41506, 0.13596, 0.04537]  # Starting position
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


    # target_pressure = 20.0  # Target pressure value
    target_pressure = 17.0  # Target pressure value

    # pressure_threshold = 10.0  # Values below this are set to 0
    pressure_threshold = 5.0  # Values below this are set to 0

    align_tolerance = 2.0  # If |P_E - P_W| < 5, stop rotating
    z_tolerance = 2.0  # If |pressure_mean - target_pressure| < 2, maintain z
   
    # === 경향성 추적을 위한 히스토리 설정 ===
    history_size = 10  # 최근 N개의 값을 저장하여 경향성 계산
    P_E_history = []  # P_E 값의 히스토리
    P_W_history = []  # P_W 값의 히스토리
    trend_weight = 0.5  # 경향성 가중치 (0.0~1.0, 높을수록 경향성에 더 의존)
   
    while 1:
        # Get pressure data
        rospy.sleep(0.05)  # Small delay to allow pressure data to update
        pressure_avg = P_help.four_pressure
       
        # Filter pressure data: set values <= 10 to 0
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
       
        # === 경향성 추적: 히스토리에 현재 값 추가 ===
        P_E_history.append(P_E)
        P_W_history.append(P_W)
        
        # 히스토리 크기 제한
        if len(P_E_history) > history_size:
            P_E_history.pop(0)
            P_W_history.pop(0)
        
        # === 경향성 계산 (변화율) ===
        P_E_trend = 0.0  # P_E의 변화율 (양수면 증가, 음수면 감소)
        P_W_trend = 0.0  # P_W의 변화율 (양수면 증가, 음수면 감소)
        trend_threshold = 0.5  # 트렌드가 의미있을 최소 변화량
        
        if len(P_E_history) >= 3:  # 최소 3개 이상의 데이터가 있어야 경향성 계산 가능
            # 최근 값들의 평균 변화율 계산
            recent_window = min(5, len(P_E_history))  # 최근 5개 또는 전체 사용
            recent_P_E = P_E_history[-recent_window:]
            recent_P_W = P_W_history[-recent_window:]
            
            # 최근 절반과 이전 절반의 평균 차이로 경향성 계산
            if len(recent_P_E) >= 2:
                mid_point = len(recent_P_E) // 2
                P_E_old_avg = np.mean(recent_P_E[:mid_point])
                P_E_new_avg = np.mean(recent_P_E[mid_point:])
                P_E_trend = P_E_new_avg - P_E_old_avg
                
                P_W_old_avg = np.mean(recent_P_W[:mid_point])
                P_W_new_avg = np.mean(recent_P_W[mid_point:])
                P_W_trend = P_W_new_avg - P_W_old_avg
        
        # === 현재 값 기반 방향 결정 ===
        P_diff_current = P_E - P_W
        current_based_direction = 0
        if abs(P_diff_current) >= align_tolerance:
            if P_diff_current > 0:
                current_based_direction = 1  # CCW
            else:
                current_based_direction = -1  # CW
        
        # === 트렌드 기반 방향 결정 (적응적) ===
        # P_W가 감소하거나 P_E가 증가하면 → CCW (1)
        # P_W가 증가하거나 P_E가 감소하면 → CW (-1)
        trend_based_direction = 0
        if abs(P_E_trend) >= trend_threshold or abs(P_W_trend) >= trend_threshold:
            # P_E가 증가하는 경향이 있으면 CCW
            # P_W가 감소하는 경향이 있으면 CCW
            if P_E_trend > trend_threshold or P_W_trend < -trend_threshold:
                trend_based_direction = 1  # CCW
            # P_E가 감소하는 경향이 있으면 CW
            # P_W가 증가하는 경향이 있으면 CW
            elif P_E_trend < -trend_threshold or P_W_trend > trend_threshold:
                trend_based_direction = -1  # CW
        
        # === 현재 값과 트렌드를 적응적으로 결합하여 최종 방향 결정 ===
        # 트렌드가 명확하면 트렌드를 우선, 그렇지 않으면 현재 값 사용
        if trend_based_direction != 0:
            # 트렌드가 있으면 트렌드와 현재 값을 가중 평균
            # 트렌드가 강할수록 더 많이 반영
            trend_strength = min(abs(P_E_trend), abs(P_W_trend)) if (abs(P_E_trend) > 0 and abs(P_W_trend) > 0) else max(abs(P_E_trend), abs(P_W_trend))
            adaptive_weight = min(trend_weight * (1.0 + trend_strength / 5.0), 0.8)  # 최대 0.8까지
            
            combined_signal = (1.0 - adaptive_weight) * current_based_direction + adaptive_weight * trend_based_direction
            align_direction = int(np.sign(combined_signal)) if abs(combined_signal) > 0.1 else trend_based_direction
        else:
            # 트렌드가 없거나 약하면 현재 값 기반으로 결정
            align_direction = current_based_direction
        
        # 최종 차이값 계산 (디버그용)
        P_diff = abs(P_diff_current)
       
        # Determine z direction based on average pressure
        # pressure_mean < 20: move down (direction=1)
        # pressure_mean > 20: move up (direction=-1)
        # If |pressure_mean - target_pressure| < z_tolerance, maintain z
        pressure_diff = abs(pressure_mean - target_pressure)
        if pressure_diff < z_tolerance:
            z_direction = 0  # maintain z (within tolerance)
        elif pressure_mean < target_pressure:
            z_direction = 1  # move down
        else:  # pressure_mean > target_pressure
            z_direction = -1  # move up
       
        # Lateral movement is fixed: always move right (-y direction, 0.005)
        T_later = adaptHelp.get_Tmat_TranlateInY(direction=-1)
       
        # Align rotation (only if needed)
        if align_direction != 0:
            T_align = adaptHelp.get_Tmats_RotationAtX(direction=align_direction)
        else:
            T_align = np.eye(4)  # No rotation
       
        # Normal movement (z direction, only if needed)
        if z_direction != 0:
            T_normalMove = adaptHelp.get_Tmat_TranlateInZ(direction=z_direction)
        else:
            T_normalMove = np.eye(4)  # No z movement
       
        # Combine transformations: lateral --> align --> normal
        T_move = T_later @ T_align @ T_normalMove
       
        # Move to new pose adaptively
        measuredCurrPose = rtde_help.getCurrentPose()
        deltaPose = adaptHelp.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        rtde_help.goToPoseAdaptive(deltaPose)
       
        # Get current y position to check if we've reached the target
        currentPose = rtde_help.getCurrentPose()
        current_y = currentPose.pose.position.y
       
        # Debug print
        print(f"Y: {current_y:.5f} (target: {positionA_y_end:.5f}), Pressure: {pressure_filtered}, P_E: {P_E:.2f}, P_W: {P_W:.2f}, P_diff: {P_diff:.2f}, Mean: {pressure_mean:.2f}, Mean_diff: {pressure_diff:.2f}, Align: {align_direction}, Z: {z_direction}, Trend_E: {P_E_trend:.3f}, Trend_W: {P_W_trend:.3f}, TrendDir: {trend_based_direction}, CurrDir: {current_based_direction}")


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



