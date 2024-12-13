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
from enum import Enum
from colorama import Fore, Back, Style
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
CASTLE_STATE = ["e2e4","e7e5","g1f3","b8c6","f1e2","d7d6"]
PROMOTION_STATE=["e2e4", "e7e5", "f2f4", "d7d6", "f4e5", "f7f6", "e5f6", "b8c6", "f6g7", "g8f6" ]
CAPTURE_STATE=["e2e4", "e7e5", "g1f3", "f8a3"]


class ChessGame:
    def __init__(self, stockfish_path):
        self.stockfish = Stockfish(stockfish_path)
        self.stockfish.set_skill_level(2)  # engine skill level (0-20)
        self.moves_played = []

        rospy.init_node('chess_game_node')
        self.pub = rospy.Publisher('/move_to_make', ChessState, queue_size=10)
        rospy.loginfo("ROS node initialized and publisher set up.")
    def display_board(self):
        board_visual = self.stockfish.get_board_visual()
        colored_board = ""
        for char in board_visual:
            if char.islower():  
                colored_board += Fore.GREEN + char + Style.RESET_ALL
            elif char.isupper(): 
                colored_board += Fore.WHITE + char + Style.RESET_ALL
            else:
                colored_board += char 

        print(colored_board)
    def make_move(self, move):
        """Make a move and update the game state."""
        if self.stockfish.is_move_correct(move):
            
            self.stockfish.make_moves_from_current_position([move])
            self.moves_played.append(move)
            print(f"Move {move} played.")
            return True
        else:
            print(f"Invalid move: '{move}'")
            return False

    def get_best_move(self):
        """Get the best move recommended by Stockfish."""
        return self.stockfish.get_best_move()

    def evaluate_position(self):
        """Evaluate the current board position."""
        return self.stockfish.get_evaluation()
    
    def publish_move(self, move):
        """Publish the move to the /move_to_make topic."""
        rospy.loginfo("Publishing move...")
        msg = ChessState()
        msg.chess_move = move
        msg.is_capture = self.stockfish.will_move_be_a_capture(move).value
        msg.piece_to_move = self.stockfish.get_what_is_on_square(move[:2]).value
        captured_piece = self.stockfish.get_what_is_on_square(move[2:4])
        if captured_piece is not None:
            msg.captured_piece = captured_piece.value
        self.moves_played.append(move)
        self.pub.publish(msg)
        self.stockfish.make_moves_from_current_position([move])
        rospy.loginfo(f"Published move: {move}")
    # def publish_checkmate(self):
    #     """Publish a checkmate message to the /move_to_make topic."""
    #     rospy.loginfo("Publishing checkmate...")
    #     msg = ChessState()
    #     msg.is_checkmate = True
    #     self.pub.publish(msg)
    #     rospy.loginfo("Published checkmate.")

    def start_game(self):
        """Start a game of chess with Stockfish."""
        print("Starting a new game of chess!")
        #self.stockfish.set_position(CASTLE_STATE    )
        self.stockfish.set_position(PROMOTION_STATE)

        while not rospy.is_shutdown():
            print("\nCurrent board:")
            self.display_board()

            # is game over?
            evaluation = self.evaluate_position()
            if evaluation["type"] == "mate":
                print("Checkmate! Game over.")
                # self.publish_checkmate()
                break
            elif evaluation["value"] == 0:
                print("Stalemate! Game over.")
                break

            # player's turn
            player_move = input("Enter your move (e.g., e2e4): ").strip()
            if not self.stockfish.is_move_correct(player_move):
                print("Invalid move. Try again.")
                continue
            # if we want the robot to make our moves also
            # then uncomment this
            self.publish_move(player_move)

            # stockfish's turn
            stockfish_move = self.get_best_move()
            if stockfish_move is not None:
                print(f"Stockfish plays: {stockfish_move}")
                if self.stockfish.is_move_correct(stockfish_move):
                    self.publish_move(stockfish_move)
                    # self.make_move(stockfish_move)
            else:
                print("No valid moves left for Stockfish. Game over.")
                break


if __name__ == "__main__":
    stockfish_path = "/home/edg/catkin_ws/src/pushpull_suctioncup_106a/src/stockfish_14_linux_x64_avx2/stockfish_14_linux_x64_avx2/stockfish_14_x64_avx2"
    game = ChessGame(stockfish_path)
    game.start_game()

