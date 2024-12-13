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
from stockfish import Stockfish
from datetime import datetime
import numpy as np
import time
import scipy
import pickle
from pushpull_suctioncup_106a.msg import ChessState

from netft_utils.srv import *
from suction_cup.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int8
import geometry_msgs.msg

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp


def game_loop():
  rospy.init_node('gameplay_node')

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2

  DUTYCYCLE_100 = 100
  DUTYCYCLE_30 = 30
  DUTYCYCLE_0 = 0
  CHECKMATE = "mate"
  WHITE_TO_PLAY = True

  np.set_printoptions(precision=4)
  stockfish = Stockfish(path="/home/edg/catkin_ws/src/pushpull_suctioncup_106a/src/stockfish_14_linux_x64_avx2/stockfish_14_linux_x64_avx2/stockfish_14_x64_avx2")

  pub = rospy.Publisher('/move_to_make', ChessState, queue_size=1)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
  syncPub.publish(SYNC_RESET)
  evaluation = stockfish.get_evaluation()
  moves_played = ["e2e4", "e7e5", "f1c4", "a7a6", "g1h3", "h7h6", "e1g1"]
  try:
    while evaluation["type"] != CHECKMATE and evaluation["value"] != 0:
        input("Press enter to continue")
        robot_move = stockfish.get_best_move()
        print(f"Robot move: {robot_move}")
        sp, ep = robot_move[:2], robot_move[2:]
        msg = ChessState()
        msg.chess_move = robot_move
        msg.is_capture = stockfish.will_move_be_a_capture(robot_move).name
        msg.piece_to_move = stockfish.get_what_is_on_square(sp).name
        pub.publish(msg)
        stockfish.make_moves_from_current_position([robot_move])

    
  except ValueError as e:
    print(e)
  


if __name__ == '__main__':  
    game_loop()
    