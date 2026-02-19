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




def main(args):


  deg2rad = np.pi / 180.0


  np.set_printoptions(precision=4)


  # controller node
  rospy.init_node('edg_experiment')


  # Setup helper functions
  rtde_help = rtdeHelp(125)
  adaptHelp = adaptMotionHelp(d_lat=0.0010, dw=0.57, d_z=0.0010) #lateral --> align --> normal = sliding right --> rolling --> moving down
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
  PULL_STATE = 1
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
#   positionA = [0.48383, -0.02173, 0.07531]  # Starting position
#   positionA = [0.58678, 0.01299, 0.02846]  # Starting position
  # positionA = [0.48143, -0.02173, 0.07531]  # for toy
  positionA = [0.53565, -0.12553, 0.01161]  # for paper



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


    # === Lateral sliding based only on pressure direction ===
    # step size is adaptHelp.d_lat (set from d_lat=0.0010 above)
    target_grasp_pressure = 50.0  # mean of 4 channels to trigger grasp (PULL)
    stable_count_required = 20  # threshold를 연속으로 넘는 최소 횟수
    stable_count = 0

    while 1:
        rospy.sleep(0.05)  # Small delay to allow pressure data to update

        # Get raw pressure data (array-like, shape (4,))
        pressure_avg = np.array(P_help.four_pressure).ravel()

        # Thresholding (<=10 -> 0) and mean based on thresholded values
        pressure = pressure_avg.copy()
        pressure[pressure <= 10.0] = 0.0
        pressure_mean = float(np.mean(pressure))

        # Check grasp condition:
        #  - 평균 압력이 target_grasp_pressure 이상인 상태가
        #  - stable_count_required 회 이상 연속으로 유지되면 grasp 시퀀스 실행
        if pressure_mean >= target_grasp_pressure:
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= stable_count_required:
            print(f"Grasp condition reached stably ({stable_count} loops), mean pressure (thresholded) = {pressure_mean:.2f}")

            # === Deformation: use servoL to move down by specified deformation ===
            # Read current position at threshold condition
            currentPose = rtde_help.getCurrentPose()
            
            # Get deformation argument (mm) and convert to meters
            deformation_mm = getattr(args, "deformation", 3.0)
            deformation_m = deformation_mm * 1e-3
            step_z = adaptHelp.d_z_normal
            n_steps_down = int(np.round(deformation_m / step_z)) if step_z > 0 else 0
            
            print(f"Applying deformation: {deformation_mm} mm (downward in {n_steps_down} steps) from Z={currentPose.pose.position.z:.6f}m")
            
            # Use servoL to move down (consistent with lateral sliding)
            for step_idx in range(max(n_steps_down, 0)):
                T_down = adaptHelp.get_Tmat_TranlateInZ(direction=1)  # direction=1: move down
                measuredCurrPose = rtde_help.getCurrentPose()
                deltaPose_down = adaptHelp.get_PoseStamped_from_T_initPose(T_down, measuredCurrPose)
                rtde_help.goToPoseAdaptive(deltaPose_down)
                rospy.sleep(0.05)  # Small delay between steps
            
            final_z = rtde_help.getCurrentPose().pose.position.z
            print(f"Final Z after deformation: {final_z:.6f}m")

            # After reaching deformation depth, wait 2 seconds (still in PUSH state)
            rospy.sleep(3.0)

            # === Grasp: switch to PULL and hold suction for 2 seconds ===
            print("Switching to PULL state for grasp...")
            msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
            PushPull_pub.publish(msg)
            rospy.sleep(3.0)

            # === Lift: use servoL to move up by same deformation + extra 15cm ===
            # Read current position (after deformation)
            currentPose_after_deform = rtde_help.getCurrentPose()
            
            # Move up by deformation distance + extra lift (15cm)
            extra_lift = 0.15  # 15cm
            total_lift = deformation_m + extra_lift
            n_steps_up = int(np.round(total_lift / step_z)) if step_z > 0 else 0
            
            print(f"Lifting: {total_lift*1000:.1f}mm (upward in {n_steps_up} steps) from Z={currentPose_after_deform.pose.position.z:.6f}m")
            
            # Use servoL to move up (consistent with deformation)
            for step_idx in range(max(n_steps_up, 0)):
                T_up = adaptHelp.get_Tmat_TranlateInZ(direction=-1)  # direction=-1: move up
                measuredCurrPose = rtde_help.getCurrentPose()
                deltaPose_up = adaptHelp.get_PoseStamped_from_T_initPose(T_up, measuredCurrPose)
                rtde_help.goToPoseAdaptive(deltaPose_up)
                rospy.sleep(0.05)  # Small delay between steps
            
            final_z_lift = rtde_help.getCurrentPose().pose.position.z
            print(f"Final Z after lift: {final_z_lift:.6f}m")

            # Stop suction (OFF_STATE) before finishing
            print("Stopping suction (OFF_STATE)...")
            msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            rospy.sleep(0.1)
            
            # Stop pressure sampling and finish
            P_help.stopSampling()
            print("============ Python UR_Interface demo complete!")
            return

        # Channel unit vectors: -45, 45, 135, 225 deg
        n = 4
        e = np.zeros((n, 2))
        for k in range(n):
            alpha_k = (k * (360.0 / n)) - 45.0
            e[k, 0] = np.cos(np.deg2rad(alpha_k))
            e[k, 1] = np.sin(np.deg2rad(alpha_k))

        # Weighted sum
        v = np.array([0.0, 0.0])
        for k in range(n):
            v = v + pressure[k] * e[k, :]

        # Normalize
        nv = np.linalg.norm(v)
        if nv == 0 or not np.isfinite(nv):
            # No meaningful direction from pressure; skip motion this cycle
            print(f"No valid lateral direction (pressure={pressure_avg}), skipping step.")
            if rospy.is_shutdown():
                msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
                PushPull_pub.publish(msg)
                P_help.stopSampling()
                return
            continue

        v = v / nv  # unit vector in lateral plane

        # Lateral step in v direction with step size d_lat
        d_lat = adaptHelp.d_lat
        dx = d_lat * v[0]
        dy = d_lat * v[1]

        T_later = adaptHelp.get_Tmat_TranlateInBodyF([dx, dy, 0.0])

        # Move to new pose adaptively using only lateral motion
        measuredCurrPose = rtde_help.getCurrentPose()
        deltaPose = adaptHelp.get_PoseStamped_from_T_initPose(T_later, measuredCurrPose)
        rtde_help.goToPoseAdaptive(deltaPose)

        # Debug print
        print(f"Pressure raw: {pressure_avg}, mean: {pressure_mean:.2f}, v: {v}, step: ({dx:.6f}, {dy:.6f})")

        if rospy.is_shutdown():
            # Stop push before returning
            msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            P_help.stopSampling()
            return

    # If we exit the loop without grasp (e.g., shutdown), clean up
    P_help.stopSampling()
    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--deformation', type=float, default=3, help='Deformation (mm) to apply downward before grasp')
  parser.add_argument('--material', type=str, default="paper", help='object to test')

  cli_args = parser.parse_args()
  main(cli_args)
