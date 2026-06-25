#!/usr/bin/env python
import rospy, tf, numpy as np, copy, pickle, time
from math import pi, floor
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull
from suction_cup.srv import Enable
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp


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


   # ========== ROS 초기화 및 헬퍼 ==========
   rospy.init_node('suction_cup')
   P_help = P_CallbackHelp(psensor_num=args.ch); rospy.sleep(0.5)
   rtde_help = rtdeHelp(125); rospy.sleep(0.5)
   file_help = fileSaveHelp()
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
   if args.ch == 3: default_yaw = pi/2 - 30*pi/180
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
       P_help.setNowAsOffset()


       input("Press <Enter> to go normal to get engage point")
       engage_z = disengagePosition_init[2] - args.deformation*1e-3
       with open(file_help.ResultSavingDirectory+'/engage_z.p', 'wb') as f:
           pickle.dump(engage_z, f)


       input("Press <Enter> to start to data collection")
      
       center_position = copy.deepcopy(disengagePosition_init)
      
       # 반지름: 0mm부터 1mm씩 증가 (중심에서 x 방향 평행이동)
       radii = np.arange(-6, 11, 4)  # [-5, 0, 5, 10]
      
       # yaw: 0°→… 반시계(CCW), radius 전 복귀는 시계(CW) 방향으로 단계 이동
       yaw_deg_list = np.arange(0, 360, 60)  # [0, 60, 120, ..., 300] CCW
       yaw_step_deg = yaw_deg_list[1] - yaw_deg_list[0] if len(yaw_deg_list) > 1 else 60
      
       for radius_mm in radii:
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
              
               args.pressure_avg = P_help.four_pressure
               args.P_vac = abs(P_help.P_vac)
              
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

           # 다음 radius 전: 300°→240°→…→0° 시계(CW) 방향으로 단계 복귀
           # (0°로 직접 goToPose하면 짧은 경로로 +60° CCW 돌아 팔이 꼬일 수 있음)
           if yaw_deg_list[-1] != 0:
               print(f"  Resetting yaw CW to 0° at radius {radius_mm}mm")
               for reset_yaw_deg in range(int(yaw_deg_list[-1] - yaw_step_deg), -1, -int(yaw_step_deg)):
                   reset_orientation = tf.transformations.quaternion_from_euler(
                       default_yaw - reset_yaw_deg*pi/180, pi, 0, 'szxy')
                   reset_pose = rtde_help.getPoseObj(disengagePosition_r, reset_orientation)
                   print(f"    CW reset yaw: {reset_yaw_deg}°")
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
   parser.add_argument('--deformation', type=float, default=6.3)
   parser.add_argument('--ch', type=int, default=3)
   parser.add_argument('--corner', type=int, default=180)
   parser.add_argument('--material', type=int, default=0)
   args = parser.parse_args()
   main(args)


