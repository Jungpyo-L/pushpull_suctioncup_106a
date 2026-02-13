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


# === PI Controller Class ===
class PIController:
    def __init__(self, Kp=1.0, Ki=0.1, integral_limit=None, output_limit=None):
        """
        PI Controller
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            integral_limit: Maximum absolute value for integral term (anti-windup)
            output_limit: Maximum absolute value for output
        """
        self.Kp = Kp
        self.Ki = Ki
        self.integral = 0.0
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.prev_error = 0.0
    
    def update(self, error, dt=0.05):
        """
        Update PI controller
        Args:
            error: Current error (setpoint - current_value)
            dt: Time step (default: 0.05s, matching rospy.sleep(0.05))
        Returns:
            output: Control output
        """
        # Proportional term
        P_term = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        
        # Anti-windup: limit integral term
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        
        I_term = self.Ki * self.integral
        
        # Total output
        output = P_term + I_term
        
        # Limit output
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)
        
        self.prev_error = error
        return output
    
    def reset(self):
        """Reset integral term"""
        self.integral = 0.0
        self.prev_error = 0.0


def main():


  deg2rad = np.pi / 180.0


  np.set_printoptions(precision=4)


  # controller node
  rospy.init_node('edg_experiment')


  # Setup helper functions
  rtde_help = rtdeHelp(125)
  adaptHelp = adaptMotionHelp(d_lat=0.0015, dw=0.57, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down
# adaptHelp = adaptMotionHelp(d_lat=0.005, dw=0.5, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down
  
  # === d_w 동적 조정을 위한 변수 설정 ===
  initial_dw_deg = 0.57  # 초기 d_w 값 (도 단위)
  initial_dw_rad = initial_dw_deg * np.pi / 180.0  # 라디안으로 변환
  dw_change_deg = 0.03  # align 변경 시 d_w 변화량 (도 단위)
  dw_change_rad = dw_change_deg * np.pi / 180.0  # 라디안으로 변환
  prev_align_direction = 0  # 이전 align_direction 값
  align_repeat_count = 0  # 같은 align_direction 값이 반복된 횟수
  align_repeat_threshold = 2  # 초기값으로 복귀하기 위한 반복 횟수

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

    # align_tolerance와 z_tolerance는 이제 PI 컨트롤러의 deadzone으로 대체됨
    # align_tolerance = 2.0  # If |P_E - P_W| < 5, stop rotating (사용 안 함)
    # z_tolerance = 2.0  # If |pressure_mean - target_pressure| < 2, maintain z (사용 안 함)
   
    # === PI Controller 설정 ===
    # Z 방향 제어용 PI 컨트롤러
    # 오차: target_pressure - pressure_mean
    # 출력: z 방향 제어 신호 (양수면 아래로, 음수면 위로)
    z_pi_controller = PIController(
        Kp=0.1,  # Proportional gain (조정 필요)
        Ki=0.01,  # Integral gain (조정 필요)
        integral_limit=50.0,  # Anti-windup limit
        output_limit=1.0  # 출력 제한 (-1 ~ 1)
    )
    
    # 회전(Align) 제어용 PI 컨트롤러
    # 오차: P_E - P_W (목표는 0, 즉 P_E = P_W)
    # 출력: 회전 방향 제어 신호 (양수면 CCW, 음수면 CW)
    align_pi_controller = PIController(
        Kp=0.2,  # Proportional gain (조정 필요)
        Ki=0.02,  # Integral gain (조정 필요)
        integral_limit=20.0,  # Anti-windup limit
        output_limit=1.0  # 출력 제한 (-1 ~ 1)
    )
   
    # === 경향성 추적을 위한 히스토리 설정 (PI 컨트롤러 사용 시 선택적) ===
    # PI 컨트롤러가 적분 항을 자체적으로 관리하므로 히스토리는 선택적으로 사용 가능
    # history_size = 10  # 최근 N개의 값을 저장하여 경향성 계산
    # P_E_history = []  # P_E 값의 히스토리
    # P_W_history = []  # P_W 값의 히스토리
    # trend_weight = 0.5  # 경향성 가중치 (0.0~1.0, 높을수록 경향성에 더 의존)
   
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
       
        # === 경향성 추적: 히스토리에 현재 값 추가 (PI 컨트롤러는 자체 적분 항을 사용하므로 선택적) ===
        # PI 컨트롤러가 적분 항을 자체적으로 관리하므로 히스토리는 선택적으로 사용 가능
        # P_E_history.append(P_E)
        # P_W_history.append(P_W)
        # 
        # # 히스토리 크기 제한
        # if len(P_E_history) > history_size:
        #     P_E_history.pop(0)
        #     P_W_history.pop(0)
        
        # === PI Controller를 사용한 회전(Align) 제어 ===
        # 오차: P_E - P_W (목표는 0, 즉 P_E = P_W가 되도록)
        P_diff_current = P_E - P_W
        align_error = P_diff_current  # 목표값 0과의 차이
        
        # PI 컨트롤러 업데이트 (dt = 0.05초, rospy.sleep(0.05)와 동일)
        align_output = align_pi_controller.update(align_error, dt=0.05)
        
        # PI 출력을 방향으로 변환 (dead zone 적용)
        align_deadzone = 0.1  # 작은 출력은 무시
        if abs(align_output) < align_deadzone:
            align_direction = 0  # 회전 없음
        else:
            align_direction = int(np.sign(align_output))  # 1: CCW, -1: CW
        
        # 최종 차이값 계산 (디버그용)
        P_diff = abs(P_diff_current)
       
        # === d_w 동적 조정 로직 ===
        # align_direction이 변경되었는지 확인 (-1 <-> 1 변경)
        if prev_align_direction != 0 and align_direction != 0:
            if prev_align_direction != align_direction:
                # align_direction이 변경됨 (-1에서 1로 또는 1에서 -1로)
                # d_w를 0.03도만큼 변경 (라디안으로 변환하여 적용)
                adaptHelp.dw += dw_change_rad
                align_repeat_count = 0  # 반복 카운트 리셋
                print(f"Align changed: {prev_align_direction} -> {align_direction}, d_w adjusted to: {adaptHelp.dw * 180.0 / np.pi:.4f} deg")
            elif prev_align_direction == align_direction:
                # 같은 align_direction 값이 반복됨
                align_repeat_count += 1
                if align_repeat_count >= align_repeat_threshold:
                    # 2번 반복되면 초기값으로 복귀
                    adaptHelp.dw = initial_dw_rad
                    align_repeat_count = 0  # 카운트 리셋
                    print(f"Align repeated {align_repeat_threshold} times, d_w reset to initial: {initial_dw_deg:.4f} deg")
        
        # 현재 align_direction을 이전 값으로 저장
        prev_align_direction = align_direction
       
        # === PI Controller를 사용한 Z 방향 제어 ===
        # 오차: target_pressure - pressure_mean
        # pressure_mean이 target_pressure보다 작으면 오차가 양수 → 아래로 이동 (direction=1)
        # pressure_mean이 target_pressure보다 크면 오차가 음수 → 위로 이동 (direction=-1)
        z_error = target_pressure - pressure_mean
        
        # PI 컨트롤러 업데이트 (dt = 0.05초, rospy.sleep(0.05)와 동일)
        z_output = z_pi_controller.update(z_error, dt=0.05)
        
        # PI 출력을 방향으로 변환 (dead zone 적용)
        z_deadzone = 0.1  # 작은 출력은 무시
        pressure_diff = abs(z_error)  # 디버그용
        if abs(z_output) < z_deadzone:
            z_direction = 0  # z 유지
        else:
            z_direction = int(np.sign(z_output))  # 1: 아래로, -1: 위로
       
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
        print(f"Y: {current_y:.5f} (target: {positionA_y_end:.5f}), Pressure: {pressure_filtered}, P_E: {P_E:.2f}, P_W: {P_W:.2f}, P_diff: {P_diff:.2f}, Mean: {pressure_mean:.2f}, Mean_diff: {pressure_diff:.2f}, Align: {align_direction} (PI_out: {align_output:.3f}, err: {align_error:.2f}), Z: {z_direction} (PI_out: {z_output:.3f}, err: {z_error:.2f}), d_w: {current_dw_deg:.4f}deg")


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


