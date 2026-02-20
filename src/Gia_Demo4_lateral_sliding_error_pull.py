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
  
  # === File saving helper ===
  file_help = fileSaveHelp()
  
  # === Data logging service ===
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False)
  rospy.sleep(1)
  file_help.clearTmpFolder()
 
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
  # positionA = [0.50684, -0.02715, 0.01155]  # for paper
  # positionA = [0.50777, -0.00597, 0.07356]  # for toy
  positionA = [0.50800, 0.05516, 0.04611]  # for glue
  # positionA = [0.51277, 0.04676, 0.02826]  # for acrylic plate
  # positionA = [0.51572, 0.06649, 0.01193]  # for pcb
  # positionA = [0.53469, -0.02315, 0.17258]  # for round_jar

  

  # Calculate positionA_true by adding xoffset (mm) to y coordinate (second element) in meters
  xoffset_m = getattr(args, "xoffset", 0) * 1e-3  # Convert mm to meters
  positionA_true = [positionA[0]-xoffset_m, positionA[1] , positionA[2]]

  positionA_y_end = (0.01299 - 0.08)  # Target y position (10cm from start: 0.02501 + 0.08 = 0.10501)
  orientationA = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA_true, orientationA)




  # try block so that we can have a keyboard exception
  try:


    input("Press <Enter> to go to pose A (positionA_true with deformation)")
    
    # Get deformation argument (mm) and convert to meters
    deformation_mm = getattr(args, "deformation", 3.0)
    deformation_m = deformation_mm * 1e-3
    
    # Calculate positionA with deformation (move down by deformation)
    positionA_with_deform = [positionA_true[0], 
                             positionA_true[1], 
                             positionA_true[2] - deformation_m]
    poseA_with_deform = rtde_help.getPoseObj(positionA_with_deform, orientationA)
    
    print(f"Moving to positionA_true with deformation: {deformation_mm} mm downward")
    rtde_help.goToPose(poseA_with_deform)
    rospy.sleep(1)
    print("poseA (with deformation): ", rtde_help.getCurrentPose())
    
    # === Initialize data collection lists ===
    # Store data for each iteration (including initial position)
    data_positions = []  # List of [x, y, z] positions
    data_v_vectors = []  # List of v vectors [vx, vy]
    data_pressure_avg = []  # List of pressure averages
    data_pressure_raw = []  # List of raw pressure arrays
    data_iteration = []  # Iteration number
    data_timestamps = []  # List of timestamps (seconds since start)
    
    # Record start time for relative timestamps
    timestamp_start_time = rospy.Time.now().to_sec()
    
    # Save initial position (positionA)
    currentPose_init = rtde_help.getCurrentPose()
    data_positions.append([currentPose_init.pose.position.x, 
                          currentPose_init.pose.position.y, 
                          currentPose_init.pose.position.z])
    data_v_vectors.append([0.0, 0.0])  # No v vector at initial position
    data_pressure_avg.append(0.0)  # No pressure data yet
    data_pressure_raw.append([0.0, 0.0, 0.0, 0.0])  # No pressure data yet
    data_iteration.append(0)  # Initial iteration
    data_timestamps.append(0.0)  # Start time (relative timestamp = 0)


    # Start pressure sampling
    P_help.startSampling()
    rospy.sleep(0.5)
    P_help.setNowAsOffset()
    rospy.sleep(0.5)

    # Start data logging
    dataLoggerEnable(True)
    rospy.sleep(0.2)

    # Start pull (PULL ON)
    print("Starting pull...")
    msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
    PushPull_pub.publish(msg)
    rospy.sleep(0.1)


    input("Press <Enter> to start surface following")


    # === Lateral sliding based only on pressure direction ===
    # step size is adaptHelp.d_lat (set from d_lat=0.0010 above)
    target_grasp_pressure = 80.0  # mean of 4 channels to trigger grasp (PULL)
    stable_count_required = 20  # threshold를 연속으로 넘는 최소 횟수
    stable_count = 0
    grasp_flag = False  # Flag to track if grasp condition is met
    start_time = time.time()  # Record start time for timeout check
    timeout_duration = 10.0  # 10 seconds timeout

    while 1:
        # Check timeout: if 10 seconds have passed without reaching grasp condition
        elapsed_time = time.time() - start_time
        if elapsed_time >= timeout_duration and not grasp_flag:
            print(f"Timeout reached ({timeout_duration}s) without reaching grasp condition. Resetting...")
            grasp_flag = False
            
            # === Stop servoL mode before switching to moveL ===
            print("Stopping servoL mode...")
            rtde_help.stopAtCurrPoseAdaptive()
            rospy.sleep(0.5)  # Wait for servoL to fully stop
            
            # Get current pose and switch to moveL mode
            currentPose_timeout = rtde_help.getCurrentPose()
            currentPosition_timeout = [currentPose_timeout.pose.position.x,
                                      currentPose_timeout.pose.position.y,
                                      currentPose_timeout.pose.position.z]
            currentOrientation_timeout = [currentPose_timeout.pose.orientation.x,
                                          currentPose_timeout.pose.orientation.y,
                                          currentPose_timeout.pose.orientation.z,
                                          currentPose_timeout.pose.orientation.w]
            poseCurrent_timeout = rtde_help.getPoseObj(currentPosition_timeout, currentOrientation_timeout)
            
            # Move to current position using moveL to activate control script in moveL mode
            print("Switching to moveL mode...")
            rtde_help.goToPose(poseCurrent_timeout, speed=0.1, acc=0.1)
            rospy.sleep(0.5)  # Wait for moveL to complete and control script to be ready
            
            # Save collected data before timeout
            iteration_num = len(data_iteration)
            current_timestamp = rospy.Time.now().to_sec() - timestamp_start_time
            data_positions.append([currentPose_timeout.pose.position.x,
                                  currentPose_timeout.pose.position.y,
                                  currentPose_timeout.pose.position.z])
            # Use last calculated v or [0,0] if not available
            if len(data_v_vectors) > 0:
                last_v = data_v_vectors[-1]
            else:
                last_v = [0.0, 0.0]
            data_v_vectors.append(last_v)
            data_pressure_avg.append(pressure_mean)
            data_pressure_raw.append(pressure_avg.tolist() if isinstance(pressure_avg, np.ndarray) else list(pressure_avg))
            data_iteration.append(iteration_num)
            data_timestamps.append(current_timestamp)
            
            # Save all collected data to mat file
            print("Saving collected data to mat file (timeout)...")
            # Stop data logging before saving
            dataLoggerEnable(False)
            rospy.sleep(0.1)
            args.data_positions = np.array(data_positions)
            args.data_v_vectors = np.array(data_v_vectors)
            args.data_pressure_avg = np.array(data_pressure_avg)
            args.data_pressure_raw = np.array(data_pressure_raw)
            args.data_iteration = np.array(data_iteration)
            args.data_timestamps = np.array(data_timestamps)
            args.positionA = positionA
            args.positionA_true = positionA_true
            args.positionGrasp = np.array([])  # Empty array instead of None for failed case
            
            xoffset_val = getattr(args, "xoffset", 0)
            file_help.saveDataParams(args, appendTxt=f'Demo4SlidingError_pull_material_{args.material}_xoffset_{xoffset_val}_failed')
            file_help.clearTmpFolder()
            
            # Set PULL_STATE to OFF_STATE (0)
            print("Turning off pull...")
            msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            rospy.sleep(0.1)
            
            # === Move to position offset from positionA (x-15cm, y+15cm, z+10cm) ===
            offset_distance = 0.15  # 15cm in meters
            positionOffset = [positionA[0] - offset_distance, 
                             positionA[1] + offset_distance, 
                             positionA[2] + 0.10]
            poseOffset = rtde_help.getPoseObj(positionOffset, orientationA)
            print(f"Moving to offset position from positionA: {positionOffset}")
            rtde_help.goToPose(poseOffset, speed=0.1, acc=0.1)
            rospy.sleep(1)
            print("Reached offset position")
            
            # Stop pressure sampling and exit
            P_help.stopSampling()
            print("============ Python UR_Interface demo complete!")
            return

        rospy.sleep(0.05)  # Small delay to allow pressure data to update

        # Get raw pressure data (array-like, shape (4,))
        pressure_avg = np.array(P_help.four_pressure).ravel()

        # Thresholding (<=10 -> 0) and mean based on thresholded values
        pressure = pressure_avg.copy()
        pressure = -pressure
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
            grasp_flag = True  # Set grasp_flag to True when condition is met
            
            # === Stop servoL mode immediately when grasp condition is met ===
            print("Stopping servoL mode...")
            rtde_help.stopAtCurrPoseAdaptive()
            rospy.sleep(0.5)  # Wait longer for servoL to fully stop
            
            # Turn off pull (PULL_STATE to OFF_STATE)
            print("Turning off pull...")
            msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            rospy.sleep(0.2)
            # Save current end effector position (like positionA)
            
            currentPose_at_grasp = rtde_help.getCurrentPose()
            positionGrasp = [currentPose_at_grasp.pose.position.x, 
                             currentPose_at_grasp.pose.position.y, 
                             currentPose_at_grasp.pose.position.z]
            
            # Add current position at grasp to data collection
            iteration_num = len(data_iteration)
            current_timestamp = rospy.Time.now().to_sec() - timestamp_start_time
            data_positions.append([currentPose_at_grasp.pose.position.x,
                                  currentPose_at_grasp.pose.position.y,
                                  currentPose_at_grasp.pose.position.z])
            # Use last calculated v or [0,0] if not available
            if len(data_v_vectors) > 0:
                last_v = data_v_vectors[-1]
            else:
                last_v = [0.0, 0.0]
            data_v_vectors.append(last_v)
            data_pressure_avg.append(pressure_mean)
            data_pressure_raw.append(pressure_avg.tolist() if isinstance(pressure_avg, np.ndarray) else list(pressure_avg))
            data_iteration.append(iteration_num)
            data_timestamps.append(current_timestamp)
            
            # Save all collected data to mat file with positionGrasp
            print("Saving collected data to mat file...")
            # Stop data logging before saving
            dataLoggerEnable(False)
            rospy.sleep(0.1)
            args.data_positions = np.array(data_positions)
            args.data_v_vectors = np.array(data_v_vectors)
            args.data_pressure_avg = np.array(data_pressure_avg)
            args.data_pressure_raw = np.array(data_pressure_raw)
            args.data_iteration = np.array(data_iteration)
            args.data_timestamps = np.array(data_timestamps)
            args.positionA = positionA
            args.positionA_true = positionA_true
            args.positionGrasp = positionGrasp
            
            xoffset_val = getattr(args, "xoffset", 0)
            file_help.saveDataParams(args, appendTxt=f'Demo4SlidingError_pull_material_{args.material}_xoffset_{xoffset_val}_success')
            file_help.clearTmpFolder()
            print(f"Grasp condition reached stably ({stable_count} loops), mean pressure (thresholded) = {pressure_mean:.2f}")
            input("Press <Enter> to go to offset position...")

            # === Switch from servoL to moveL mode before moving to offset ===
            # First, stop servoL mode
            rtde_help.stopAtCurrPoseAdaptive()
            rospy.sleep(0.5)  # Wait for servoL to fully stop
            
            # Get current pose and switch to moveL mode by moving to current position
            # This ensures RTDE control script is running in moveL mode
            currentPose_switch = rtde_help.getCurrentPose()
            currentPosition_switch = [currentPose_switch.pose.position.x,
                                      currentPose_switch.pose.position.y,
                                      currentPose_switch.pose.position.z]
            currentOrientation_switch = [currentPose_switch.pose.orientation.x,
                                         currentPose_switch.pose.orientation.y,
                                         currentPose_switch.pose.orientation.z,
                                         currentPose_switch.pose.orientation.w]
            poseCurrent = rtde_help.getPoseObj(currentPosition_switch, currentOrientation_switch)
            
            # Move to current position using moveL to activate control script in moveL mode
            print("Switching to moveL mode...")
            rtde_help.goToPose(poseCurrent, speed=0.1, acc=0.1)
            rospy.sleep(0.5)  # Wait for moveL to complete and control script to be ready

            # === Move to position offset from positionA (x+15cm, y+15cm) ===
            offset_distance = 0.15  # 15cm in meters
            positionOffset = [positionA[0] - offset_distance, 
                             positionA[1] + offset_distance, 
                             positionA[2] + 0.10]
            poseOffset = rtde_help.getPoseObj(positionOffset, orientationA)
            print(f"Moving to offset position from positionA: {positionOffset}")
            rtde_help.goToPose(poseOffset, speed=0.1, acc=0.1)
            rospy.sleep(1)
            
            # Wait for user to press Enter at offset position
            input("Press <Enter> to return to positionGrasp...")
            
            # === Ensure we're in moveL mode before returning to positionGrasp ===
            # Get current pose and ensure moveL mode is active
            currentPose_before_return = rtde_help.getCurrentPose()
            currentPosition_before_return = [currentPose_before_return.pose.position.x,
                                             currentPose_before_return.pose.position.y,
                                             currentPose_before_return.pose.position.z]
            currentOrientation_before_return = [currentPose_before_return.pose.orientation.x,
                                                currentPose_before_return.pose.orientation.y,
                                                currentPose_before_return.pose.orientation.z,
                                                currentPose_before_return.pose.orientation.w]
            poseCurrent_before_return = rtde_help.getPoseObj(currentPosition_before_return, currentOrientation_before_return)
            
            # Move to current position using moveL to ensure control script is ready
            print("Ensuring moveL mode is active...")
            rtde_help.goToPose(poseCurrent_before_return, speed=0.1, acc=0.1)
            rospy.sleep(0.5)  # Wait for moveL to complete and control script to be ready
            
            # === Return to positionGrasp ===
            # Get orientation from currentPose_at_grasp
            orientationGrasp = [currentPose_at_grasp.pose.orientation.x,
                               currentPose_at_grasp.pose.orientation.y,
                               currentPose_at_grasp.pose.orientation.z,
                               currentPose_at_grasp.pose.orientation.w]
            poseGrasp = rtde_help.getPoseObj(positionGrasp, orientationGrasp)
            print(f"Returning to positionGrasp: {positionGrasp}")
            rtde_help.goToPose(poseGrasp, speed=0.1, acc=0.1)
            rospy.sleep(1.5)  # Wait longer for moveL to complete
            
            # Wait 1 second at positionGrasp
            print("Waiting 1 second at positionGrasp...")
            rospy.sleep(1.0)
            
            # === Grasp: switch to PULL and hold suction for 3 seconds ===
            # Read current position (already at positionGrasp with deformation applied at start)
            currentPose = rtde_help.getCurrentPose()
            print(f"Current position at grasp: Z={currentPose.pose.position.z:.6f}m")
            
            print("Switching to PULL state for grasp...")
            msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
            PushPull_pub.publish(msg)
            rospy.sleep(3.0)

            # === Lift: use moveL to move up by deformation + extra 20cm ===
            # Read current position (at positionGrasp)
            currentPose_after_deform = rtde_help.getCurrentPose()
            
            # Get deformation argument (mm) and convert to meters
            deformation_mm = getattr(args, "deformation", 3.0)
            deformation_m = deformation_mm * 1e-3
            
            # Move up by deformation distance + extra lift (20cm)
            extra_lift = 0.20  # 20cm
            total_lift = deformation_m + extra_lift
            
            # Calculate target position (move up by total_lift)
            target_position_lift = [currentPose_after_deform.pose.position.x,
                                   currentPose_after_deform.pose.position.y,
                                   currentPose_after_deform.pose.position.z + total_lift]
            target_orientation_lift = [currentPose_after_deform.pose.orientation.x,
                                      currentPose_after_deform.pose.orientation.y,
                                      currentPose_after_deform.pose.orientation.z,
                                      currentPose_after_deform.pose.orientation.w]
            poseLift = rtde_help.getPoseObj(target_position_lift, target_orientation_lift)
            
            print(f"Lifting: {total_lift*1000:.1f}mm (upward) from Z={currentPose_after_deform.pose.position.z:.6f}m")
            
            # Use moveL to move up (single smooth motion instead of steps)
            rtde_help.goToPose(poseLift, speed=0.1, acc=0.1)
            rospy.sleep(1.5)  # Wait longer for motion to complete
            
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
        
        # Get current position after movement
        currentPose_after_move = rtde_help.getCurrentPose()
        
        # Collect data for this iteration
        iteration_num = len(data_iteration)  # Current iteration number
        current_timestamp = rospy.Time.now().to_sec() - timestamp_start_time
        data_positions.append([currentPose_after_move.pose.position.x,
                              currentPose_after_move.pose.position.y,
                              currentPose_after_move.pose.position.z])
        data_v_vectors.append([v[0], v[1]])  # Store normalized v vector
        data_pressure_avg.append(pressure_mean)
        data_pressure_raw.append(pressure_avg.tolist() if isinstance(pressure_avg, np.ndarray) else list(pressure_avg))
        data_iteration.append(iteration_num)
        data_timestamps.append(current_timestamp)

        # Debug print
        print(f"Pressure raw: {pressure_avg}, mean: {pressure_mean:.2f}, v: {v}, step: ({dx:.6f}, {dy:.6f})")

        if rospy.is_shutdown():
            # === Stop servoL mode before switching to moveL ===
            print("Stopping servoL mode...")
            rtde_help.stopAtCurrPoseAdaptive()
            rospy.sleep(0.5)  # Wait for servoL to fully stop
            
            # Get current pose and switch to moveL mode
            currentPose_shutdown = rtde_help.getCurrentPose()
            currentPosition_shutdown = [currentPose_shutdown.pose.position.x,
                                       currentPose_shutdown.pose.position.y,
                                       currentPose_shutdown.pose.position.z]
            currentOrientation_shutdown = [currentPose_shutdown.pose.orientation.x,
                                           currentPose_shutdown.pose.orientation.y,
                                           currentPose_shutdown.pose.orientation.z,
                                           currentPose_shutdown.pose.orientation.w]
            poseCurrent_shutdown = rtde_help.getPoseObj(currentPosition_shutdown, currentOrientation_shutdown)
            
            # Move to current position using moveL to activate control script in moveL mode
            print("Switching to moveL mode...")
            rtde_help.goToPose(poseCurrent_shutdown, speed=0.1, acc=0.1)
            rospy.sleep(0.5)  # Wait for moveL to complete and control script to be ready
            
            # Save collected data before shutdown
            if len(data_positions) > 0:
                print("Saving collected data to mat file (shutdown)...")
                # Stop data logging before saving
                dataLoggerEnable(False)
                rospy.sleep(0.1)
                args.data_positions = np.array(data_positions)
                args.data_v_vectors = np.array(data_v_vectors)
                args.data_pressure_avg = np.array(data_pressure_avg)
                args.data_pressure_raw = np.array(data_pressure_raw)
                args.data_iteration = np.array(data_iteration)
                args.data_timestamps = np.array(data_timestamps)
                args.positionA = positionA
                args.positionA_true = positionA_true
                args.positionGrasp = np.array([])  # Empty array instead of None for failed case
                
                xoffset_val = getattr(args, "xoffset", 0)
                file_help.saveDataParams(args, appendTxt=f'Demo4SlidingError_pull_material_{args.material}_xoffset_{xoffset_val}_failed')
                file_help.clearTmpFolder()
            
            # Stop pull before moving to offset
            print("Turning off pull...")
            msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            rospy.sleep(0.1)
            
            # === Move to position offset from positionA (x-15cm, y+15cm, z+10cm) ===
            offset_distance = 0.15  # 15cm in meters
            positionOffset = [positionA[0] - offset_distance, 
                             positionA[1] + offset_distance, 
                             positionA[2] + 0.10]
            poseOffset = rtde_help.getPoseObj(positionOffset, orientationA)
            print(f"Moving to offset position from positionA: {positionOffset}")
            rtde_help.goToPose(poseOffset, speed=0.1, acc=0.1)
            rospy.sleep(1)
            print("Reached offset position")
            
            P_help.stopSampling()
            return

    # If we exit the loop without grasp (e.g., shutdown), clean up
    # === Stop servoL mode before switching to moveL ===
    print("Stopping servoL mode...")
    rtde_help.stopAtCurrPoseAdaptive()
    rospy.sleep(0.5)  # Wait for servoL to fully stop
    
    # Get current pose and switch to moveL mode
    currentPose_exit = rtde_help.getCurrentPose()
    currentPosition_exit = [currentPose_exit.pose.position.x,
                            currentPose_exit.pose.position.y,
                            currentPose_exit.pose.position.z]
    currentOrientation_exit = [currentPose_exit.pose.orientation.x,
                               currentPose_exit.pose.orientation.y,
                               currentPose_exit.pose.orientation.z,
                               currentPose_exit.pose.orientation.w]
    poseCurrent_exit = rtde_help.getPoseObj(currentPosition_exit, currentOrientation_exit)
    
    # Move to current position using moveL to activate control script in moveL mode
    print("Switching to moveL mode...")
    rtde_help.goToPose(poseCurrent_exit, speed=0.1, acc=0.1)
    rospy.sleep(0.5)  # Wait for moveL to complete and control script to be ready
    
    # Save collected data before exit
    if len(data_positions) > 0:
        print("Saving collected data to mat file (loop exit)...")
        # Stop data logging before saving
        dataLoggerEnable(False)
        rospy.sleep(0.1)
        args.data_positions = np.array(data_positions)
        args.data_v_vectors = np.array(data_v_vectors)
        args.data_pressure_avg = np.array(data_pressure_avg)
        args.data_pressure_raw = np.array(data_pressure_raw)
        args.data_iteration = np.array(data_iteration)
        args.data_timestamps = np.array(data_timestamps)
        args.positionA = positionA
        args.positionA_true = positionA_true
        args.positionGrasp = np.array([])  # Empty array instead of None for failed case
        
        xoffset_val = getattr(args, "xoffset", 0)
        file_help.saveDataParams(args, appendTxt=f'Demo4SlidingError_pull_material_{args.material}_xoffset_{xoffset_val}_failed')
        file_help.clearTmpFolder()
    
    # Stop pull before moving to offset
    print("Turning off pull...")
    msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
    PushPull_pub.publish(msg)
    rospy.sleep(0.1)
    
    # === Move to position offset from positionA (x-15cm, y+15cm, z+10cm) ===
    offset_distance = 0.15  # 15cm in meters
    positionOffset = [positionA[0] - offset_distance, 
                     positionA[1] + offset_distance, 
                     positionA[2] + 0.10]
    poseOffset = rtde_help.getPoseObj(positionOffset, orientationA)
    print(f"Moving to offset position from positionA: {positionOffset}")
    rtde_help.goToPose(poseOffset, speed=0.1, acc=0.1)
    rospy.sleep(1)
    print("Reached offset position")
    
    P_help.stopSampling()
    dataLoggerEnable(False)
    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    dataLoggerEnable(False)
    return
  except KeyboardInterrupt:
    dataLoggerEnable(False)
    return


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--deformation', type=float, default=3, help='Deformation (mm) to apply downward before grasp')
  parser.add_argument('--material', type=str, default="paper", help='object to test')
  parser.add_argument('--xoffset', type=float, default=0, help='X offset (mm) to add to positionA y coordinate')

  cli_args = parser.parse_args()
  main(cli_args)
