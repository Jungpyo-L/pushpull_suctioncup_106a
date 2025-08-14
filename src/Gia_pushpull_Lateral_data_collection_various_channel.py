#!/usr/bin/env python
# 통합 suction + push/pull 메시지 구조 예시
import rospy, tf, numpy as np, copy, pickle, time
from math import pi, floor
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull
from suction_cup.srv import Enable
from helperFunction.SuctionP_callback_helper import P_CallbackHelp #####different from suction pkg -> so changed little but mostly from suctioncup pkg
from helperFunction.FT_callback_helper import FT_CallbackHelp #####same as suction pkg
from helperFunction.fileSaveHelper import fileSaveHelp #####same as suction pkg
from helperFunction.rtde_helper import rtdeHelp #####different from suction pkg -> same as suction pkg bc I copied and pasted
from helperFunction.adaptiveMotion import adaptMotionHelp #####same as suction pkg bc I copied and pasted

def main(args):
    # ==== 상수 정의 ====
    DUTYCYCLE_100 = 100
    DUTYCYCLE_0 = 0
    SYNC_RESET = 0
    SYNC_START = 1
    SYNC_STOP = 2
    PULL_STATE = 1
    PUSH_STATE = 2
    OFF_STATE = 0

    F_normalThres = [args.normalForce, args.normalForce + 0.5]
    args.normalForce_thres = F_normalThres

    # ==== ROS 초기화 ====
    rospy.init_node('suction_cup')
    FT_help = FT_CallbackHelp(); rospy.sleep(0.5)
    P_help = P_CallbackHelp(); rospy.sleep(0.5)
    rtde_help = rtdeHelp(125); rospy.sleep(0.5)
    file_help = fileSaveHelp()
    adpt_help = adaptMotionHelp(dw=0.5, d_lat=0.5e-3, d_z=0.1e-3)
    rospy.sleep(0.5)

    # TCP Offset 설정
    rtde_help.setTCPoffset([0, 0, 0.150, 0, 0, 0])
    if args.ch in (5,6):
        rtde_help.setTCPoffset([0, 0, 0.150 + 0.02 - 0.0008, 0, 0, 0])

    # Publisher/Service 설정
    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10); rospy.sleep(0.5)
    msg = PushPull()
    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)
    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
    dataLoggerEnable(False); rospy.sleep(1)
    file_help.clearTmpFolder()
    datadir = file_help.ResultSavingDirectory

    # ==== 초기 Pose 설정 ====
    if args.corner == 180:
        disengagePosition_init = [0.6092, -.275, 0.0180]
    elif args.corner == 270:
        disengagePosition_init = [0.555, 0.100, 0.0170]
    elif args.corner == 90:
        disengagePosition_init = [0.6165, -.2258, 0.0170]
    args.disengagePosition_init = disengagePosition_init

    if args.ch == 3: default_yaw = pi/2 - 60*pi/180
    if args.ch == 4: default_yaw = pi/2 - 45*pi/180
    if args.ch == 5: default_yaw = pi/2 - 90*pi/180
    if args.ch == 6: default_yaw = pi/2 - 60*pi/180

    setOrientation = tf.transformations.quaternion_from_euler(pi/2,pi,0,'szxy')
    disEngagePose = rtde_help.getPoseObj(disengagePosition_init, setOrientation)

    try:
        # ==== Disengage Pose로 이동 ====
        input("Press <Enter> to go DisengagePose")
        rtde_help.goToPose(disEngagePose)

        # 힘/압력 센서 오프셋
        P_help.startSampling(); rospy.sleep(1)
        FT_help.setNowAsBias()
        P_help.setNowAsOffset()

        # ==== Engage point 찾기 ====
        input("Press <Enter> to go normal to get engage point")
        if args.zHeight:
            engage_z = disengagePosition_init[2] - args.deformation*1e-3
        else:
            targetPose = rtde_help.getCurrentPose()
            farFlag = True
            F_normal = FT_help.averageFz_noOffset
            msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_0
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

        # ==== 데이터 수집 루프 ====
        input("Press <Enter> to start to data collection")
        startAngleFlag = True
        SuctionFlag = False
        xoffsets = np.arange(args.xoffset, 5, 1)

        for j in xoffsets:
            disengagePosition = copy.deepcopy(disengagePosition_init)
            disengagePosition[0] += j*0.001
            engagePosition = copy.deepcopy(disengagePosition)
            engagePosition[2] = engage_z

            for i in range(round(args.angle/5)+1):
                args.theta = round((pi/36*i)*180/pi)
                if args.startAngle > args.theta and startAngleFlag:
                    continue
                startAngleFlag = False

                # Disengage Pose 이동
                targetOrientation = tf.transformations.quaternion_from_euler(default_yaw - 5*pi/180*i,pi,0,'szxy')
                targetPose_init = rtde_help.getPoseObj(disengagePosition, targetOrientation)
                rtde_help.goToPose(targetPose_init)

                # PushPull 상태: 흡착 OFF
                msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_0
                PushPull_pub.publish(msg)
                syncPub.publish(SYNC_RESET)

                P_help.startSampling(); rospy.sleep(0.3)
                P_help.setNowAsOffset()

                # Engage Pose 이동 + 흡착 ON
                targetPose = rtde_help.getPoseObj(engagePosition, targetOrientation)
                rtde_help.goToPose(targetPose)
                msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
                PushPull_pub.publish(msg)

                # 로깅 시작
                dataLoggerEnable(True)
                rospy.sleep(0.2)
                syncPub.publish(SYNC_START)
                rospy.sleep(1)

                # 데이터 취득
                P_init = P_help.four_pressure
                F_normal = FT_help.averageFz_noOffset
                args.normalForceActual = F_normal
                args.pressure_avg = P_init
                P_vac = P_help.P_vac

                if all(np.array(P_init) < P_vac) and i == 0:
                    SuctionFlag = True
                else:
                    SuctionFlag = False

                # 로깅 종료
                syncPub.publish(SYNC_STOP)
                rospy.sleep(0.1)
                msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_0
                PushPull_pub.publish(msg)
                rtde_help.goToPose(targetPose_init)
                rospy.sleep(0.1)
                dataLoggerEnable(False)

                # 파일 저장
                file_help.saveDataParams(args,
                    appendTxt=f'jp_lateral_corner_{args.corner}_xoffset_{args.xoffset}_theta_{args.theta}_material_{args.material}')
                file_help.clearTmpFolder()
                P_help.stopSampling()
                rospy.sleep(0.1)

            if not SuctionFlag:
                for i in range(floor(args.angle/90)):
                    targetOrientation = tf.transformations.quaternion_from_euler(default_yaw + pi/2*(i+1),pi,0,'szxy')
                    targetPose = rtde_help.getPoseObj(disengagePosition, targetOrientation)
                    rtde_help.goToPose(targetPose)

        # 마지막 Disengage 복귀 및 종료
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
    parser.add_argument('--xoffset', type=int, default=-4)
    parser.add_argument('--angle', type=int, default=360)
    parser.add_argument('--startAngle', type=int, default=0)
    parser.add_argument('--normalForce', type=float, default=1.5)
    parser.add_argument('--deformation', type=float, default=4.0)
    parser.add_argument('--zHeight', type=bool, default=True)
    parser.add_argument('--ch', type=int, default=4)
    parser.add_argument('--corner', type=int, default=180)
    parser.add_argument('--material', type=int, default=0)
    args = parser.parse_args()
    main(args)
