#!/usr/bin/env python
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

    # === 초기 disengage position 설정 ===
    # chamber 위치에 따라 기준 변경 (아래 조건은 첫번째 코드와 동일)
    if args.corner == 180:
        disengagePosition_init = [0.6092, -.275, 0.0180]
    elif args.corner == 270:
        disengagePosition_init = [0.555, 0.100, 0.0170]
    elif args.corner == 90:
        disengagePosition_init = [0.6165, -.2258, 0.0170]
    args.disengagePosition_init = disengagePosition_init

    # ch별 중심 yaw 기준 오프셋 적용
    if args.ch == 3: default_yaw = pi/2 - 60*pi/180
    # if args.ch == 4: default_yaw = pi/2 - 45*pi/180
    if args.ch == 4: default_yaw = pi/2 - 45*pi/180 + 45*pi/180

    if args.ch == 5: default_yaw = pi/2 - 90*pi/180
    if args.ch == 6: default_yaw = pi/2 - 60*pi/180

    setOrientation = tf.transformations.quaternion_from_euler(pi/2,pi,0,'szxy')
    disEngagePose = rtde_help.getPoseObj(disengagePosition_init, setOrientation)

    try:
        input("Press <Enter> to go DisengagePose")
        rtde_help.goToPose(disEngagePose)
        P_help.startSampling(); rospy.sleep(1)
        FT_help.setNowAsBias()
        P_help.setNowAsOffset()

        input("Press <Enter> to go normal to get engage point")
        if args.zHeight:
            engage_z = disengagePosition_init[2] - args.deformation*1e-3
        else:
            targetPose = rtde_help.getCurrentPose()
            farFlag = True
            F_normal = FT_help.averageFz_noOffset
            msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_0
            PushPull_pub.publish(msg)
            while farFlag:
                if F_normal > -F_normalThres[0]:
                    T_move = adpt_help.get_Tmat_TranlateInZ(direction=1)
                elif F_normal < -F_normalThres[1]:
                    T_move = adpt_help.get_Tmat_TranlateInZ(direction=-1)
                else:
                    farFlag = False
                    rtde_help.stopAtCurrPoseAdaptive()
                    args.normalForceUsed = F_normal
                    break
                targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, targetPose)
                rtde_help.goToPoseAdaptive(targetPose, time=0.1)
                F_normal = FT_help.averageFz_noOffset
            engage_z = rtde_help.getCurrentPose().pose.position.z
            rtde_help.goToPose(disEngagePose)
            with open(file_help.ResultSavingDirectory+'/engage_z.p', 'wb') as f:
                pickle.dump(engage_z, f)

        input("Press <Enter> to start to data collection")
        startAngleFlag = True
        SuctionFlag = False

        # ====== xoffset(0, 3, 6) 반복 ======
        xoffsets = [0, 3, 6]
        for j in xoffsets:
            disengagePosition = copy.deepcopy(disengagePosition_init)
            disengagePosition[0] += j*0.001
            engagePosition = copy.deepcopy(disengagePosition)
            engagePosition[2] = engage_z

            # ==== 각도(yaw) 반복 ====
            for i in range(round(args.angle/15)+1):
                args.theta = round((pi/12*i)*180/pi)
                if args.startAngle > args.theta and startAngleFlag:
                    continue
                startAngleFlag = False

                # Disengage Pose 이동: chamber와 yaw 기준 포함(첫번째 코드와 같음)
                targetOrientation = tf.transformations.quaternion_from_euler(default_yaw - 15*pi/180*i, pi, 0, 'szxy')
                targetPose_init = rtde_help.getPoseObj(disengagePosition, targetOrientation)
                rtde_help.goToPose(targetPose_init)

                # 흡착 OFF로 pushpull 설정 (메시지 타입은 pushpull_suctioncup_106a/PushPull)
                msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_0
                PushPull_pub.publish(msg)
                syncPub.publish(SYNC_RESET)

                # 센서 샘플링/오프셋
                P_help.startSampling(); rospy.sleep(0.3)
                P_help.setNowAsOffset()

                # Engage Pose 이동 및 2초 대기, 흡착 ON
                targetPose = rtde_help.getPoseObj(engagePosition, targetOrientation)
                rtde_help.goToPose(targetPose)
                rospy.sleep(2.0) ####
                msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
                PushPull_pub.publish(msg)

                # 데이터 로깅 시작/취득/정지
                dataLoggerEnable(True)
                rospy.sleep(0.2)
                syncPub.publish(SYNC_START)
                rospy.sleep(5)

                # 데이터 취득 (압력, 힘, vacuum 값)
                P_init = P_help.four_pressure
                F_normal = FT_help.averageFz_noOffset
                args.normalForceActual = F_normal
                args.pressure_avg = P_init
                P_vac = P_help.P_vac

                if all(np.array(P_init) < P_vac) and i == 0:
                    SuctionFlag = True
                else:
                    SuctionFlag = False

                syncPub.publish(SYNC_STOP)
                rospy.sleep(0.1)

                msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_0
                PushPull_pub.publish(msg)
                rtde_help.goToPose(targetPose_init)
                rospy.sleep(0.1)
                dataLoggerEnable(False)
                file_help.saveDataParams(args,
                    appendTxt=f'Gia_lateral_corner_{args.corner}_xoffset_{j}_theta_{args.theta}_material_{args.material}')
                file_help.clearTmpFolder()
                P_help.stopSampling()
                rospy.sleep(0.1)

            if not SuctionFlag:
                for i in range(floor(args.angle/90)):
                    targetOrientation = tf.transformations.quaternion_from_euler(default_yaw + pi/2*(i+1), pi, 0, 'szxy')
                    targetPose = rtde_help.getPoseObj(disengagePosition, targetOrientation)
                    rtde_help.goToPose(targetPose)

        # ===== 실험 종료 및 뒷정리 =====
        rtde_help.goToPose(disEngagePose)
        dataLoggerEnable(False)
        P_help.stopSampling()
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--xoffset', type=int, default=0)
    parser.add_argument('--angle', type=int, default=360)
    parser.add_argument('--startAngle', type=int, default=0)
    parser.add_argument('--normalForce', type=float, default=1.5)
    parser.add_argument('--deformation', type=float, default=5.0)
    parser.add_argument('--zHeight', type=bool, default=True)
    parser.add_argument('--ch', type=int, default=4)
    parser.add_argument('--corner', type=int, default=180)
    parser.add_argument('--material', type=int, default=0)
    args = parser.parse_args()
    main(args)
