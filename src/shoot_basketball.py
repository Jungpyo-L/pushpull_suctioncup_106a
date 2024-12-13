#!/usr/bin/env python

try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

from calendar import month_abbr
import os, sys
import string
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat
import numpy as np
from helperFunction.rtde_helper import rtdeHelp
from pushpull_suctioncup_106a.msg import PushPull

msg = PushPull()


def shoot_basketball(rtde_help,publisher):
  deg2rad = np.pi / 180.0
  DUTYCYCLE_100 = 100
  DUTYCYCLE_30 = 30
  DUTYCYCLE_0 = 0
  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2
  
  # pick up the ball in position A
  FT_SimulatorOn = False
  np.set_printoptions(precision=4)
  positionA = [0.502, 0.2, 0.280]
  positionA[1] -= 0.3
  shooting_orientation = tf.transformations.quaternion_from_euler(np.pi - 75*deg2rad, 0 , -np.pi/8,'sxyz')
  #   shooting_orientation = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi/8,'sxyz')

  shooting_pose = rtde_help.getPoseObj(positionA, [shooting_orientation[0], shooting_orientation[1], shooting_orientation[2], shooting_orientation[3]])
  try:
    rtde_help.goToPoseAdaptive(shooting_pose)
    rospy.sleep(0.4)
    msg.pwm = DUTYCYCLE_100
    msg.state = 1
    publisher.publish(msg)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  
  

if __name__ == '__main__':
    try:    
        rospy.init_node('shoot_basketball')
        print("test")
        rtde_help = rtdeHelp()
        print("test1")
        if ros_enabled:
            print("test2")
            shooter = shoot_basketball(rtde_help)  # i have no idea what an RTDE helper is but i assume it's important
  
        else:
            print("ROS not enabled. Cannot proceed.")
    except rospy.ROSInterruptException:
        pass
    
  # TODO: publisher to topic goal_point has a message of the type Point x y z of the target hoop.
  # TODO: we need to align the robot arm with the x position of the hoop.
  # TODO: then, we need the robot arm to rotate at a velocity necessary to give the ball enough velocity to reach the hoop
  # TODO: then, the robot arm has to extend all the way outward in the z direction
  # TODO: somewhere along the trajectory of the robot arm swing is the optimal position to release the ball such that the trajectory of the ball can reach the hoop
  # TODO: at this moment, we reverse the pressure of the suction cup and release the ball.
