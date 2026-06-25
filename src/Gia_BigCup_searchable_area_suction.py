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
   P_help = P_CallbackHelp(psensor_num=args.ch); rospy.sleep(0.5)
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
       disengagePosition_init = [0.63282, -.0457, 0.0841]
   elif args.corner == 270:
       disengagePosition_init = [0.555, 0.100, 0.0170] #25mm=0.025m --> 0.025+0.0170=0.042m
   elif args.corner == 90:
       disengagePosition_init = [0.6165, -.2258, 0.0170] #30mm=0.03m --> 0.03+0.0170=0.047m   
   args.disengagePosition_init = disengagePosition_init


   # ch별 중심 yaw 기준 오프셋 적용
   if args.ch == 3: default_yaw = pi/2 + 120*pi/180
   if args.ch == 4: default_yaw = pi/2 - 45*pi/180
   if args.ch == 5: default_yaw = pi/2 - 90*pi/180
   if args.ch == 6: default_yaw = pi/2 - 60*pi/180


   setOrientation = tf.transformations.quaternion_from_euler(pi/2,pi,0,'szxy')
   disEngagePose = rtde_help.getPoseObj(disengagePosition_init, setOrientation)


   try:
       input("Press <Enter> to go DisengagePose")
       rtde_help.goToPose(disEngagePose)
       rospy.sleep(0.1)
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
           rospy.sleep(0.1)
           with open(file_help.ResultSavingDirectory+'/engage_z.p', 'wb') as f:
               pickle.dump(engage_z, f)


       input("Press <Enter> to start to data collection")
      
       center_position = copy.deepcopy(disengagePosition_init)
      
       # 반지름: 0mm부터 1mm씩 증가 (중심에서 x 방향 평행이동)
       radii = np.arange(-6, 11, 4)  # [-5, 0, 5, 10]
      
       # yaw: 0도부터 30도씩, 330도까지
       yaw_deg_list = np.arange(0, 360, 60)  # [0, 30, 60, ..., 330]
      
       for radius_idx, radius_mm in enumerate(radii):
           args.radius = radius_mm
          
           print(f"\n=== Starting radius: {radius_mm}mm ===")
          
           # 반지름 r 위치 (xy 평행이동, z는 동일)
           disengagePosition_r = copy.deepcopy(center_position)
           disengagePosition_r[0] += radius_mm * 1e-3
          
           for yaw_deg in yaw_deg_list:
               args.theta = yaw_deg
              
               print(f"  Yaw: {yaw_deg} degrees (radius: {radius_mm}mm)")
              
               targetOrientation = tf.transformations.quaternion_from_euler(
                   default_yaw - yaw_deg*pi/180, pi, 0, 'szxy')
              
               disengagePose_r = rtde_help.getPoseObj(disengagePosition_r, targetOrientation)
              
               engagePosition_r = copy.deepcopy(disengagePosition_r)
               engagePosition_r[2] = disengagePosition_r[2] - args.deformation * 1e-3
               engagePose_r = rtde_help.getPoseObj(engagePosition_r, targetOrientation)
              
               # 1. Disengage 위치로 이동 (반지름 r, yaw 적용)
               rtde_help.goToPose(disengagePose_r)
               rospy.sleep(0.1)
              
               msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_0
               PushPull_pub.publish(msg)
               syncPub.publish(SYNC_RESET)
               rospy.sleep(0.1)
              
               P_help.startSampling()
               rospy.sleep(0.3)
               P_help.setNowAsOffset()
              
               # 2. Engage 위치로 이동 (deformation만큼 내려감)
               rtde_help.goToPose(engagePose_r)
               rospy.sleep(0.1)
              
               # 3. PULL 2초 + 데이터 저장
               msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
               PushPull_pub.publish(msg)
              
               dataLoggerEnable(True)
               rospy.sleep(0.2)
               syncPub.publish(SYNC_START)
              
               rospy.sleep(2.0)
              
               P_init = P_help.four_pressure
               F_normal = FT_help.averageFz_noOffset
               args.normalForceActual = F_normal
               args.pressure_avg = P_init
               P_vac = abs(P_help.P_vac)
              
               syncPub.publish(SYNC_STOP)
               rospy.sleep(0.1)
              
               # 4. 다시 Disengage 위치로 올라옴
               msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_0
               PushPull_pub.publish(msg)
               rtde_help.goToPose(disengagePose_r)
               rospy.sleep(0.1)
              
               args.radius = radius_mm
               args.theta = yaw_deg
               dataLoggerEnable(False)
               file_help.saveDataParams(args,
                   appendTxt=f'Gia_searchable_radius_{radius_mm}mm_yaw_{yaw_deg}deg_deformation_{args.deformation}mm_material_{args.material}')
              
               file_help.clearTmpFolder()
               P_help.stopSampling()
               rospy.sleep(0.1)

           # 다음 radius로 넘어가기 전 yaw 0° 복귀 (한 번에 점프하면 반시계 장회전 → 단계별 역방향)
           if radius_idx < len(radii) - 1 and len(yaw_deg_list) > 1:
               print(f"  Resetting yaw to 0° step-by-step (reverse sweep)")
               for reset_yaw in reversed(yaw_deg_list[:-1]):
                   print(f"    -> yaw {reset_yaw}°")
                   reset_orientation = tf.transformations.quaternion_from_euler(
                       default_yaw - reset_yaw*pi/180, pi, 0, 'szxy')
                   reset_pose = rtde_help.getPoseObj(disengagePosition_r, reset_orientation)
                   rtde_help.goToPose(reset_pose)
                   rospy.sleep(0.1)


       # ===== 실험 종료 및 뒷정리 =====
       print("Go to disengage point")
       setOrientation = tf.transformations.quaternion_from_euler(pi/2,pi,0,'szxy')
       disEngagePose = rtde_help.getPoseObj(disengagePosition_init, setOrientation)
       rtde_help.goToPose(disEngagePose)
       rospy.sleep(0.3)
      
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
   parser.add_argument('--deformation', type=float, default=6.3)
   parser.add_argument('--zHeight', type=bool, default=True)
   parser.add_argument('--ch', type=int, default=3)
   parser.add_argument('--corner', type=int, default=180)
   parser.add_argument('--material', type=int, default=0)
   args = parser.parse_args()
   main(args)


