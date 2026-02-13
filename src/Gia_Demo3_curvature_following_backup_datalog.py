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
from suction_cup.srv import Enable


from helperFunction.rtde_helper import rtdeHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp




def main():


  deg2rad = np.pi / 180.0


  np.set_printoptions(precision=4)


  # controller node
  rospy.init_node('edg_experiment')


  # Setup helper functions
  rtde_help = rtdeHelp(125)
  # dw 기본값을 0.57로 설정 (라디안)
  initial_dw_rad = 0.57  # 기본 d_w 값 (라디안)
  adaptHelp = adaptMotionHelp(d_lat=0.0013, dw=initial_dw_rad, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down
  
  # === d_w 동적 조정을 위한 변수 설정 ===
  dw_change_rad = 0.03  # align 변경 시 d_w 변화량 (라디안)
  prev_align_direction = 0  # 이전 align_direction 값
  align_repeat_count = 0  # 방향 변경 후 같은 방향이 반복된 횟수
  align_repeat_threshold = 2  # 방향 변경 후 초기값으로 복귀하기 위한 반복 횟수

  P_help = P_CallbackHelp()  # Pressure sensor helper
  file_help = fileSaveHelp()  # File save helper
  rospy.sleep(0.5)
 
  # === 데이터 로깅 서비스 설정 ===
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False)
  rospy.sleep(1)
  file_help.clearTmpFolder()
 
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
    
    # === 데이터 로깅 시작 ===
    dataLoggerEnable(True)
    rospy.sleep(0.2)
    
    # === 0.5초마다 mat 파일 저장을 위한 시간 추적 ===
    last_save_time = time.time()
    save_interval = 0.5  # 0.5초마다 저장
    save_count = 0  # 저장 횟수 카운터
    
    # === args 객체 생성 (데이터 저장용) ===
    class Args:
        pass
    args = Args()
    args.target_pressure = target_pressure
    args.pressure_threshold = pressure_threshold
    args.align_tolerance = align_tolerance
    args.z_tolerance = z_tolerance
   
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
                # d_w를 0.57 + 0.03 = 0.60으로 설정
                adaptHelp.dw = initial_dw_rad + dw_change_rad
                align_repeat_count = 0  # 반복 카운트 리셋
                print(f"Align changed: {prev_align_direction} -> {align_direction}, d_w adjusted to: {adaptHelp.dw:.4f} rad ({adaptHelp.dw * 180.0 / np.pi:.4f} deg)")
            elif prev_align_direction == align_direction and align_direction != 0:
                # 같은 방향이 반복됨
                align_repeat_count += 1
                if align_repeat_count >= align_repeat_threshold:
                    # 2번 반복되면 초기값(0.57)으로 복귀
                    adaptHelp.dw = initial_dw_rad
                    align_repeat_count = 0  # 카운트 리셋
                    print(f"Align repeated {align_repeat_threshold} times, d_w reset to initial: {initial_dw_rad:.4f} rad ({initial_dw_rad * 180.0 / np.pi:.4f} deg)")
        
        # 현재 align_direction을 이전 값으로 저장
        prev_align_direction = align_direction
       
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
        current_dw_deg = adaptHelp.dw * 180.0 / np.pi
        print(f"Y: {current_y:.5f} (target: {positionA_y_end:.5f}), Pressure: {pressure_filtered}, P_E: {P_E:.2f}, P_W: {P_W:.2f}, Δ: {delta:.2f}, dΔ/dt: {delta_first_derivative:.3f}, d²Δ/dt²: {delta_second_derivative:.3f}, Align: {align_direction}, Z: {z_direction}, d_w: {current_dw_deg:.4f}deg")

        # === 0.5초마다 mat 파일 저장 ===
        current_time = time.time()
        if current_time - last_save_time >= save_interval:
            # 데이터 로깅 정지
            dataLoggerEnable(False)
            rospy.sleep(0.1)
            
            # 현재 상태를 args에 저장
            args.current_y = current_y
            args.pressure_filtered = pressure_filtered
            args.P_E = P_E
            args.P_W = P_W
            args.P_N = P_N
            args.P_S = P_S
            args.pressure_mean = pressure_mean
            args.align_direction = align_direction
            args.z_direction = z_direction
            args.delta = delta
            args.delta_first_derivative = delta_first_derivative
            args.delta_second_derivative = delta_second_derivative
            args.current_dw_deg = current_dw_deg
            args.save_count = save_count
            
            # mat 파일 저장
            file_help.saveDataParams(args,
                appendTxt=f'curvature_following_save_{save_count:04d}')
            
            # 임시 폴더 정리
            file_help.clearTmpFolder()
            
            # 데이터 로깅 재시작
            dataLoggerEnable(True)
            rospy.sleep(0.2)
            
            # 시간 및 카운터 업데이트
            last_save_time = current_time
            save_count += 1
            print(f"Saved mat file #{save_count}")


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
    
    # === 데이터 로깅 정지 및 마지막 저장 ===
    dataLoggerEnable(False)
    rospy.sleep(0.1)
    
    # 마지막 데이터 저장
    currentPose = rtde_help.getCurrentPose()
    args.current_y = currentPose.pose.position.y
    args.save_count = save_count
    file_help.saveDataParams(args,
        appendTxt=f'curvature_following_final_save_{save_count:04d}')
    file_help.clearTmpFolder()
   
    # Stop pressure sampling
    P_help.stopSampling()

   
   
   


    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    # 예외 발생 시 데이터 로깅 정지
    try:
      dataLoggerEnable(False)
      P_help.stopSampling()
      msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
      PushPull_pub.publish(msg)
    except:
      pass
    return
  except KeyboardInterrupt:
    # 예외 발생 시 데이터 로깅 정지
    try:
      dataLoggerEnable(False)
      P_help.stopSampling()
      msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
      PushPull_pub.publish(msg)
    except:
      pass
    return  




if __name__ == '__main__':
  main()



