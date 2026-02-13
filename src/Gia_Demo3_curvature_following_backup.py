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
  # dw 기본값을 0.57도로 설정 (adaptMotionHelp는 도 단위를 받음)
  initial_dw_deg = 0.57  # 기본 d_w 값 (도 단위)
  adaptHelp = adaptMotionHelp(d_lat=0.0013, dw=initial_dw_deg, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down
  
  # === d_w 동적 조정을 위한 변수 설정 ===
  dw_change_deg = 0.03  # align 변경 시 d_w 변화량 (도 단위)
  prev_align_direction = 0  # 이전 align_direction 값
  align_repeat_count = 0  # 방향 변경 후 같은 방향이 반복된 횟수
  align_repeat_threshold = 2  # 방향 변경 후 초기값으로 복귀하기 위한 반복 횟수

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

    align_tolerance = 2.0  # If |P_E - P_W| < 2, stop rotating
    z_tolerance = 2.0  # If |pressure_mean - target_pressure| < 2, maintain z
   
    # === 2차 미분 기반 align을 위한 히스토리 설정 ===
    delta_history_size = 10  # Δ = P_E - P_W의 히스토리 크기
    delta_history = []  # Δ 값의 히스토리
    delta_first_derivative_history = []  # 1차 미분 (dΔ/dt)의 히스토리
   
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
        
        # === 2차 미분 기반 align 계산 ===
        # Step 1: Δ = P_E - P_W 계산 (1차 델타)
        delta = P_E - P_W
        P_diff = abs(delta)
        
        # Step 2: Δ 히스토리에 추가
        delta_history.append(delta)
        if len(delta_history) > delta_history_size:
            delta_history.pop(0)
        
        # Step 3: 1차 미분 계산 (dΔ/dt, 기울기)
        delta_first_derivative = 0.0
        if len(delta_history) >= 2:
            # 최근 값들의 변화율 계산 (간단한 차분법)
            recent_window = min(5, len(delta_history))
            recent_delta = delta_history[-recent_window:]
            if len(recent_delta) >= 2:
                # 최근 절반과 이전 절반의 평균 차이로 1차 미분 계산
                mid_point = len(recent_delta) // 2
                delta_old_avg = np.mean(recent_delta[:mid_point])
                delta_new_avg = np.mean(recent_delta[mid_point:])
                delta_first_derivative = delta_new_avg - delta_old_avg
        
        # Step 4: 1차 미분 히스토리에 추가
        delta_first_derivative_history.append(delta_first_derivative)
        if len(delta_first_derivative_history) > delta_history_size:
            delta_first_derivative_history.pop(0)
        
        # Step 5: 2차 미분 계산 (d²Δ/dt², 기울기의 변화율)
        delta_second_derivative = 0.0
        if len(delta_first_derivative_history) >= 2:
            # 1차 미분의 변화율 계산
            recent_window = min(5, len(delta_first_derivative_history))
            recent_first_deriv = delta_first_derivative_history[-recent_window:]
            if len(recent_first_deriv) >= 2:
                mid_point = len(recent_first_deriv) // 2
                first_deriv_old_avg = np.mean(recent_first_deriv[:mid_point])
                first_deriv_new_avg = np.mean(recent_first_deriv[mid_point:])
                delta_second_derivative = first_deriv_new_avg - first_deriv_old_avg
        
        # Step 6: 2차 미분 기반으로 align 방향 결정
        # 2차 미분 > 0: CCW (1)
        # 2차 미분 < 0: CW (-1)
        # 2차 미분 = 0 또는 tolerance 내: 회전 없음 (0)
        if abs(delta) < align_tolerance:
            align_direction = 0  # tolerance 내이면 회전 없음
        elif delta_second_derivative > 0:
            align_direction = 1  # CCW
        elif delta_second_derivative < 0:
            align_direction = -1  # CW
        else:
            # 2차 미분이 0에 가까우면 현재 델타 값으로 결정
            if delta > 0:
                align_direction = 1  # CCW
            else:
                align_direction = -1  # CW
       
        # === d_w 동적 조정 로직 ===
        # align_direction이 변경되었는지 확인 (CW <-> CCW 변경, 즉 -1 <-> 1)
        if prev_align_direction != 0 and align_direction != 0:
            if prev_align_direction != align_direction:
                # 방향이 변경됨 (CW -> CCW 또는 CCW -> CW)
                # d_w를 증가 (도 단위로 계산 후 라디안으로 변환)
                new_dw_deg = initial_dw_deg + dw_change_deg
                adaptHelp.dw = new_dw_deg * np.pi / 180.0  # 도를 라디안으로 변환
                align_repeat_count = 0  # 반복 카운트 리셋
                print(f"Align changed: {prev_align_direction} -> {align_direction}, d_w adjusted to: {new_dw_deg:.4f} deg ({adaptHelp.dw:.4f} rad)")
            elif prev_align_direction == align_direction and align_direction != 0:
                # 같은 방향이 반복됨
                align_repeat_count += 1
                if align_repeat_count >= align_repeat_threshold:
                    # 2번 반복되면 초기값으로 복귀
                    adaptHelp.dw = initial_dw_deg * np.pi / 180.0  # 도를 라디안으로 변환
                    align_repeat_count = 0  # 카운트 리셋
                    print(f"Align repeated {align_repeat_threshold} times, d_w reset to initial: {initial_dw_deg:.4f} deg ({adaptHelp.dw:.4f} rad)")
        
        # 현재 align_direction을 이전 값으로 저장
        prev_align_direction = align_direction
       
        # === 곡면 기울기 계산 (T_later + T_normalMove를 하나로 합침) ===
        # 압력 차이로부터 곡면의 기울기 추정
        dP_WE = P_W - P_E  # 서쪽-동쪽 압력 차이
        dP_SN = P_S - P_N  # 남쪽-북쪽 압력 차이
        
        # 기본 이동 거리 (라디안/deg 주의!)
        d_lat = adaptHelp.d_lat  # 횡방향 이동 거리 (미터)
        d_z = adaptHelp.d_z_normal  # 수직 이동 거리 (미터)
        
        # 곡면 기울기 기반 이동 벡터 계산
        # y 방향: 항상 오른쪽으로 이동 (-y 방향)
        dy = -d_lat  # 항상 오른쪽으로
        
        # z 방향: 압력 기반 조정
        pressure_diff = pressure_mean - target_pressure
        if abs(pressure_diff) < z_tolerance:
            dz = 0.0  # 목표 압력 범위 내면 z 유지
        else:
            # 압력이 목표보다 작으면 아래로, 크면 위로
            # 압력 차이에 비례하여 이동 거리 조정 (부드럽게)
            pressure_scale = np.clip(abs(pressure_diff) / 5.0, 0.0, 1.0)  # 최대 1.0으로 제한
            if pressure_mean < target_pressure:
                dz = d_z * pressure_scale  # 아래로
            else:
                dz = -d_z * pressure_scale  # 위로
        
        # 곡면 기울기 보정 (압력 차이로부터 기울기 추정)
        # dP_WE가 크면 곡면이 기울어져 있음 -> z 방향 보정
        # dP_SN이 크면 곡면이 앞뒤로 기울어져 있음 -> 추가 고려 가능
        slope_correction_scale = 0.3  # 기울기 보정 강도 (0~1)
        if abs(dP_WE) > pressure_threshold:
            # 압력 차이에 비례하여 z 방향 보정
            slope_z_correction = slope_correction_scale * (dP_WE / 20.0) * d_z  # 정규화 후 스케일링
            dz += slope_z_correction
        
        # 곡면 기울기를 따라가는 통합 변환 행렬 생성
        # [0, dy, dz] 방향으로 이동 (x=0, y=dy, z=dz)
        T_surface_follow = adaptHelp.get_Tmat_TranlateInBodyF([0.0, dy, dz])
       
        # === Align rotation: 2차 미분만 사용 (Step 6에서 이미 계산됨) ===
        # align_direction은 Step 6에서 이미 2차 미분 기반으로 계산되었음
        # Align rotation (only if needed)
        if align_direction != 0:
            T_align = adaptHelp.get_Tmats_RotationAtX(direction=align_direction)
        else:
            T_align = np.eye(4)  # No rotation
       
        # Combine transformations: surface_follow --> align
        # 곡면 따라가기 먼저, 그 다음 회전
        T_move = T_surface_follow @ T_align
       
        # Move to new pose adaptively
        measuredCurrPose = rtde_help.getCurrentPose()
        deltaPose = adaptHelp.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        rtde_help.goToPoseAdaptive(deltaPose)
       
        # Get current y position to check if we've reached the target
        currentPose = rtde_help.getCurrentPose()
        current_y = currentPose.pose.position.y
       
        # Debug print
        current_dw_deg = adaptHelp.dw * 180.0 / np.pi
        print(f"Y: {current_y:.5f} (target: {positionA_y_end:.5f}), Pressure: {pressure_filtered}, P_E: {P_E:.2f}, P_W: {P_W:.2f}, Δ: {delta:.2f}, dΔ/dt: {delta_first_derivative:.3f}, d²Δ/dt²: {delta_second_derivative:.3f}, Align: {align_direction}, dz: {dz:.6f}, d_w: {current_dw_deg:.4f}deg")
        
        # === 안전 체크: 회전 각도가 너무 크면 제한 ===
        max_dw_deg = 2.0  # 최대 회전 각도 (도)
        if current_dw_deg > max_dw_deg:
            adaptHelp.dw = max_dw_deg * np.pi / 180.0
            print(f"WARNING: d_w exceeded maximum ({max_dw_deg} deg), limited to {max_dw_deg} deg")


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



