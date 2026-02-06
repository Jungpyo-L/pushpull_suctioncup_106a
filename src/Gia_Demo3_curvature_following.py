#!/usr/bin/env python
"""
Pressure-based Curvature Following Controller

This controller enables a suction cup with 4 pressure sensors to continuously 
follow curved surfaces by adjusting z-height and yaw angle based on pressure 
sensor feedback.

Control Strategy:
1. Move rightward (x-direction) with configurable step size
2. Adjust z-height based on pressure differential between left and right chambers
3. Adjust yaw angle to smoothly follow surface curvature
4. Maintain contact by recovering pressure values

Mathematical Formulation:
- Pressure differential: ΔP = P_W - P_E = (P2+P3)/2 - (P0+P1)/2
- Z-height control: Δz = -α_z * ΔP (negative feedback)
- Yaw control: Δθ = -α_yaw * ΔP (negative feedback)
- Rightward motion: x_new = x_old + step_size

Where:
- P0, P1: Right side pressure sensors (East)
- P2, P3: Left side pressure sensors (West)
- α_z: Z-height control gain
- α_yaw: Yaw angle control gain
"""

import os
import rospy, tf, numpy as np, copy, pickle, time
from math import pi, floor
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull
from suction_cup.srv import Enable
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from scipy.spatial.transform import Rotation as R

class CurvatureFollowingController:
    """
    Pressure-based curvature following controller for suction cup.
    
    This controller uses pressure sensor feedback to maintain contact with
    curved surfaces while moving laterally.
    """
    
    def __init__(self, step_size=5e-3, alpha_z=0.5e-3, alpha_yaw=0.3, 
                 pressure_threshold=20.0, max_iterations=200):
        """
        Initialize curvature following controller.
        
        Parameters:
        -----------
        step_size : float
            Lateral movement step size in meters (default: 5mm)
        alpha_z : float
            Z-height control gain in m/Pa (default: 0.5e-3 m/Pa)
        alpha_yaw : float
            Yaw angle control gain in rad/Pa (default: 0.3 rad/Pa)
        pressure_threshold : float
            Minimum pressure differential to trigger control action (Pa)
        max_iterations : int
            Maximum number of control iterations
        """
        self.step_size = step_size  # Lateral step size (m)
        self.alpha_z = alpha_z      # Z-height control gain
        self.alpha_yaw = alpha_yaw  # Yaw control gain
        self.pressure_threshold = pressure_threshold
        self.max_iterations = max_iterations
        
        # Reference pressure for recovery
        self.P_ref = None
        
    def compute_pressure_differential(self, P_array):
        """
        Compute pressure differential between left and right chambers.
        
        Parameters:
        -----------
        P_array : list or np.array
            Array of 4 pressure values [P0, P1, P2, P3]
            P0, P1: Right side (East)
            P2, P3: Left side (West)
        
        Returns:
        --------
        dP : float
            Pressure differential (P_W - P_E)
        P_E : float
            Average right side pressure
        P_W : float
            Average left side pressure
        """
        P0, P1, P2, P3 = P_array
        
        # Average pressures for East (right) and West (left)
        P_E = (P1 + P0) / 2.0  # Right side average
        P_W = (P3 + P2) / 2.0   # Left side average
        
        # Pressure differential (positive when left > right)
        dP = P_W - P_E
        
        return dP, P_E, P_W
    
    def compute_z_adjustment(self, dP):
        """
        Compute z-height adjustment based on pressure differential.
        
        When right side pressure decreases (dP > 0), lower z-height.
        
        Parameters:
        -----------
        dP : float
            Pressure differential (P_W - P_E)
        
        Returns:
        --------
        dz : float
            Z-height adjustment in meters
        """
        if abs(dP) < self.pressure_threshold:
            return 0.0
        
        # Negative feedback: if right pressure is low (dP > 0), lower z
        dz = -self.alpha_z * dP
        
        # Limit maximum adjustment
        dz = np.clip(dz, -2e-3, 2e-3)  # Max 2mm adjustment per step
        
        return dz
    
    def compute_yaw_adjustment(self, dP):
        """
        Compute yaw angle adjustment based on pressure differential.
        
        When right side pressure decreases, rotate to follow curvature.
        
        Parameters:
        -----------
        dP : float
            Pressure differential (P_W - P_E)
        
        Returns:
        --------
        dtheta : float
            Yaw angle adjustment in radians
        """
        if abs(dP) < self.pressure_threshold:
            return 0.0
        
        # Negative feedback: if right pressure is low, rotate rightward
        dtheta = -self.alpha_yaw * dP * (pi / 180.0)  # Convert to radians
        
        # Limit maximum adjustment
        dtheta = np.clip(dtheta, -5.0 * pi / 180.0, 5.0 * pi / 180.0)  # Max 5 deg
        
        return dtheta
    
    def check_pressure_recovery(self, P_E, P_ref):
        """
        Check if pressure has recovered to reference level.
        
        Parameters:
        -----------
        P_E : float
            Current right side average pressure
        P_ref : float
            Reference pressure level
        
        Returns:
        --------
        recovered : bool
            True if pressure has recovered
        """
        if P_ref is None:
            return False
        
        # Check if current pressure is within threshold of reference
        recovery_threshold = 0.8  # 80% of reference pressure
        return P_E >= P_ref * recovery_threshold


def main(args):
    # === 실험 상수 정의 ===
    DUTYCYCLE_100 = 100
    DUTYCYCLE_0 = 0
    SYNC_RESET = 0
    SYNC_START = 1
    SYNC_STOP = 2
    PULL_STATE = 1
    PUSH_STATE = 2
    OFF_STATE = 0

    # Normal force threshold (chamber 수에 따라 다름 가능)
    F_normalThres = [args.normalForce, args.normalForce + 0.5]
    args.normalForce_thres = F_normalThres

    # ========== ROS 초기화 및 헬퍼 ==========
    rospy.init_node('suction_cup')
    FT_help = FT_CallbackHelp(); rospy.sleep(0.5)
    P_help = P_CallbackHelp(); rospy.sleep(0.5)
    rtde_help = rtdeHelp(125); rospy.sleep(0.5)
    file_help = fileSaveHelp()
    adpt_help = adaptMotionHelp(dw=0.5, d_lat=0.5e-3, d_z=0.1e-3)
    rospy.sleep(0.5)

    # TCP offset/chamber별 추가 z-보정
    rtde_help.setTCPoffset([0, 0, 0.150, 0, 0, 0])
    if args.ch in (5,6):
        rtde_help.setTCPoffset([0, 0, 0.150 + 0.02 - 0.0008, 0, 0, 0])

    # === PushPull 토픽/메세지 ===
    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10); rospy.sleep(0.5)
    msg = PushPull()

    # === 동기화 토픽 ===
    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)

    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
    dataLoggerEnable(False); rospy.sleep(1)
    file_help.clearTmpFolder()
    datadir = file_help.ResultSavingDirectory

    # === 초기 위치 설정 (고정값) ===
    start_position = [0.6672, -0.236, 0.0531]  # 고정 시작 위치
    
    # === 컨트롤러 파라미터 설정 ===
    # step_size: 오른쪽 이동 스텝 크기 (m)
    step_size = args.stepSize if hasattr(args, 'stepSize') else 5e-3  # 기본값: 5mm
    
    # 컨트롤러 초기화
    controller = CurvatureFollowingController(
        step_size=step_size,
        alpha_z=args.alphaZ if hasattr(args, 'alphaZ') else 0.5e-3,  # Z 제어 gain
        alpha_yaw=args.alphaYaw if hasattr(args, 'alphaYaw') else 0.3,  # Yaw 제어 gain
        pressure_threshold=args.pressureThres if hasattr(args, 'pressureThres') else 20.0,
        max_iterations=args.maxIter if hasattr(args, 'maxIter') else 200
    )
    
    # ch별 중심 yaw 기준 오프셋 적용
    if args.ch == 3: default_yaw = pi/2 - 60*pi/180
    if args.ch == 4: default_yaw = pi/2 - 45*pi/180
    if args.ch == 5: default_yaw = pi/2 - 90*pi/180
    if args.ch == 6: default_yaw = pi/2 - 60*pi/180
    
    # 초기 orientation 설정
    initial_yaw = default_yaw if 'default_yaw' in locals() else 0.0
    setOrientation = tf.transformations.quaternion_from_euler(pi/2, pi, initial_yaw, 'szxy')
    startPose = rtde_help.getPoseObj(start_position, setOrientation)

    try:
        input("Press <Enter> to go to start position")
        rtde_help.goToPose(startPose)
        rospy.sleep(0.5)
        
        # 압력 센서 샘플링 시작 및 오프셋 설정
        P_help.startSampling()
        rospy.sleep(1.0)
        FT_help.setNowAsBias()
        P_help.setNowAsOffset()
        
        # 초기 압력값 측정 (참조값 설정)
        rospy.sleep(0.3)
        P_init = P_help.four_pressure
        dP_init, P_E_init, P_W_init = controller.compute_pressure_differential(P_init)
        controller.P_ref = P_E_init  # 오른쪽 압력 참조값 설정
        
        print(f"Initial pressure - P_E: {P_E_init:.2f}, P_W: {P_W_init:.2f}, dP: {dP_init:.2f}")
        print(f"Starting curvature following with step_size: {step_size*1000:.1f}mm")
        
        input("Press <Enter> to start curvature following")
        
        # Push 상태로 설정
        msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(0.2)
        
        # 데이터 로깅 시작
        dataLoggerEnable(True)
        rospy.sleep(0.2)
        syncPub.publish(SYNC_START)
        
        # 현재 pose 가져오기
        currentPose = rtde_help.getCurrentPose()
        current_position = [
            currentPose.pose.position.x,
            currentPose.pose.position.y,
            currentPose.pose.position.z
        ]
        current_orientation = [
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        
        # 현재 yaw 각도 추출
        r = R.from_quat(current_orientation)
        euler = r.as_euler('szxy', degrees=False)
        current_yaw = euler[2]  # yaw angle
        
        iteration = 0
        pressure_history = []

        # === 좌표/행동 변화마다 MAT 저장용 (헬퍼 수정 없이 saveDataParams만 사용) ===
        # - 매 반복마다 pose가 변하므로 기본적으로 매 반복 저장된다.
        # - 필요 시 아래 조건을 바꿔 "행동(dz/dyaw) 발생 시"만 저장하도록 축소 가능.
        last_saved_position = None
        last_saved_yaw = None
        
        print("\n=== Starting Curvature Following ===")
        print(f"Step size: {step_size*1000:.1f}mm")
        print(f"Control gains - alpha_z: {controller.alpha_z*1e3:.3f} mm/Pa, alpha_yaw: {controller.alpha_yaw:.3f} rad/Pa")
        
        # 메인 제어 루프
        while iteration < controller.max_iterations and not rospy.is_shutdown():
            iteration += 1
            
            # 현재 압력값 읽기
            P_array = P_help.four_pressure
            
            # 압력 차이 계산
            dP, P_E, P_W = controller.compute_pressure_differential(P_array)
            
            # Z 높이 조정 계산
            dz = controller.compute_z_adjustment(dP)
            
            # Yaw 각도 조정 계산
            dyaw = controller.compute_yaw_adjustment(dP)
            
            # 오른쪽으로 이동 (x 방향 증가)
            new_x = current_position[0] + step_size
            new_y = current_position[1]  # y는 유지
            new_z = current_position[2] + dz  # z 높이 조정
            
            # Yaw 각도 업데이트
            new_yaw = current_yaw + dyaw
            
            # 새로운 orientation 계산
            new_orientation = tf.transformations.quaternion_from_euler(pi/2, pi, new_yaw, 'szxy')
            
            # 새로운 pose 생성
            new_position = [new_x, new_y, new_z]
            targetPose = rtde_help.getPoseObj(new_position, new_orientation)
            
            # 로봇 이동 (adaptive motion 사용)
            rtde_help.goToPoseAdaptive(targetPose, time=0.1)
            
            # 상태 출력
            if iteration % 10 == 0:
                print(f"Iteration {iteration:3d}: x={new_x:.4f}, z={new_z:.4f}, yaw={new_yaw*180/pi:.2f}deg, "
                      f"dP={dP:.2f}, P_E={P_E:.2f}, P_W={P_W:.2f}")
            
            # 압력 히스토리 저장
            pressure_history.append({
                'iteration': iteration,
                'position': copy.deepcopy(new_position),
                'yaw': new_yaw,
                'pressure': copy.deepcopy(P_array),
                'dP': dP,
                'P_E': P_E,
                'P_W': P_W,
                'dz': dz,
                'dyaw': dyaw
            })
            
            # 현재 상태 업데이트
            current_position = new_position
            current_yaw = new_yaw

            # === MAT 저장: 좌표(및 행동) 변화 시마다 저장 ===
            # Demo2(206-207)과 동일하게 file_help.saveDataParams(args, appendTxt=...)만 이용.
            pose_changed = (
                last_saved_position is None
                or np.linalg.norm(np.array(new_position) - np.array(last_saved_position)) > 0.0
                or last_saved_yaw is None
                or abs(new_yaw - last_saved_yaw) > 0.0
            )
            if pose_changed:
                # 저장할 값들을 args에 넣어 MAT로 저장되게 한다.
                args.iteration = iteration
                args.pose_xyz = np.array(new_position, dtype=float)
                args.yaw = float(new_yaw)
                args.pressure_array = np.array(P_array, dtype=float)
                args.pressure_E = float(P_E)
                args.pressure_W = float(P_W)
                args.pressure_dP = float(dP)
                args.action_dz = float(dz)
                args.action_dyaw = float(dyaw)
                args.pushpull_state = int(PUSH_STATE)

                file_help.saveDataParams(
                    args,
                    appendTxt=(
                        f'Gia_curvature_following_step{step_size*1000:.1f}mm_'
                        f'iter{iteration:04d}_material_{args.material}'
                    ),
                )

                last_saved_position = copy.deepcopy(new_position)
                last_saved_yaw = float(new_yaw)
            
            # 압력 복구 확인 (선택적 종료 조건)
            if controller.check_pressure_recovery(P_E, controller.P_ref):
                print(f"\nPressure recovered at iteration {iteration}")
                # 계속 진행할지 선택 가능
            
            # 작은 지연 (제어 주기)
            rospy.sleep(0.1)
        
        # 데이터 로깅 정지
        syncPub.publish(SYNC_STOP)
        rospy.sleep(0.1)
        dataLoggerEnable(False)
        
        # 압력 히스토리 저장
        args.pressure_history = pressure_history
        args.total_iterations = iteration
        args.final_position = current_position
        args.final_yaw = current_yaw
        
        # 데이터 저장
        file_help.saveDataParams(args,
            appendTxt=f'Gia_curvature_following_step{step_size*1000:.1f}mm_iter{iteration}_material_{args.material}')
        
        print(f"\n=== Curvature Following Completed ===")
        print(f"Total iterations: {iteration}")
        print(f"Final position: {current_position}")
        print(f"Final yaw: {current_yaw*180/pi:.2f} degrees")
        
        # Push 상태 해제
        msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        
        # 시작 위치로 복귀
        input("Press <Enter> to return to start position")
        rtde_help.goToPose(startPose)
        rospy.sleep(0.3)
        
        P_help.stopSampling()
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("\n=== Interrupted by user ===")
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        rtde_help.stopAtCurrPoseAdaptive()
        P_help.stopSampling()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Pressure-based curvature following controller')
    parser.add_argument('--normalForce', type=float, default=1.5, help='Normal force threshold')
    parser.add_argument('--deformation', type=float, default=3.0, help='Deformation (not used in this mode)')
    parser.add_argument('--zHeight', type=bool, default=True, help='Use z height')
    parser.add_argument('--ch', type=int, default=4, help='Chamber number')
    parser.add_argument('--corner', type=int, default=180, help='Corner angle')
    parser.add_argument('--material', type=int, default=0, help='Material type')
    
    # 컨트롤러 파라미터
    parser.add_argument('--stepSize', type=float, default=5e-3, 
                       help='Lateral step size in meters (default: 5mm)')
    parser.add_argument('--alphaZ', type=float, default=0.5e-3,
                       help='Z-height control gain in m/Pa (default: 0.5e-3)')
    parser.add_argument('--alphaYaw', type=float, default=0.3,
                       help='Yaw angle control gain in rad/Pa (default: 0.3)')
    parser.add_argument('--pressureThres', type=float, default=20.0,
                       help='Pressure differential threshold in Pa (default: 20.0)')
    parser.add_argument('--maxIter', type=int, default=200,
                       help='Maximum number of iterations (default: 200)')
    
    args = parser.parse_args()
    main(args)
