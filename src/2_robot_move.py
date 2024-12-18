#!/usr/bin/env python

try:
  import rospy
  import tf
  ros_enabled = True
except:
  rospy.loginfo('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False
from calendar import month_abbr
import os, sys
import string
from pushpull_suctioncup_106a.msg import PushPull, ChessState
import numpy as np
from helperFunction.rtde_helper import rtdeHelp
try:
    import rospy
    import tf
    from geometry_msgs.msg import Point
    import rtde_receive
    ros_enabled = True
except:
    rospy.loginfo("Couldn't import ROS. I assume you're running this on your laptop")
    ros_enabled = False

import time
from std_msgs.msg import Int8
from pushpull_suctioncup_106a.msg import PushPull
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp
from netft_utils.srv import StartSim
from suction_cup.srv import Enable
import random 

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from dataclasses import dataclass


# Constants
#We can update this manually by checking the center using the object_detection.py and center_position.py script
HANGING_POSITION = [0.5517035985963046, -0.06948488804144246, 0.280]
Z = 0.0151
IN_TO_M = 0.0254
DUTYCYCLE_100 = 100
SYNC_RESET = 0
PULL_STATE = 2
PUSH_STATE = 1
OFF_STATE = 0
BOTTOM, TOP = -10.5 * IN_TO_M, 19.5 * IN_TO_M
WIDTH = 13.5 * IN_TO_M
SQUARE_SIZE = 3 * IN_TO_M
FILES = dict(zip(('pr', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'), np.arange(-WIDTH, WIDTH, SQUARE_SIZE)))
RANKS = dict(zip((1, 2, 3, 4, 5, 6, 7, 8, 9, 10), np.arange(BOTTOM, TOP, SQUARE_SIZE)))

GRAVEYARD = (FILES['b'], RANKS[9])
GULAG_WHITE = (FILES['pr'], RANKS[2])
GULAG_BLACK = (FILES['pr'], RANKS[7])
PIECE_TO_FILE = {
    'p': 'b',
    'n': 'c',
    'b': 'd',
    'r': 'e',
    'q': 'f',
    'k': 'g'
}


@dataclass
class MoveCoordinates:
    pickup_x: float
    pickup_y: float
    place_x: float
    place_y: float


def listener():
    if ros_enabled:
        rospy.Subscriber('/move_to_make', ChessState, make_move)
        rospy.spin()
      
def move_to_hanging_position():
    """
    Moves the chess piece to the origin on the board.
    """
    rospy.loginfo("moving to hanging position")
    origin = rtde_help.getPoseObj(HANGING_POSITION, set_orientation)
    rtde_help.goToPose(origin)

# def move_to_graveyard():
#     rospy.loginfo("moving to graveyard")
#     graveyard = rtde_help.getPoseObj(graveyard_position, set_orientation)
#     rtde_help.goToPose(graveyard)

def pick_or_place(dx, dy, place=False, picking_up_promotion=False):
    """
    Picks up the chess piece from the board.
    """
    
    if place:
        rospy.loginfo("placing")
        position = [rtde_help.getCurrentPose().pose.position.x + dx, rtde_help.getCurrentPose().pose.position.y + dy, Z]
    elif picking_up_promotion: #added this case because picking up from side of the board has a different z value
        rospy.loginfo("picking up promotion")
        position = [HANGING_POSITION[0] + dx, HANGING_POSITION[1] + dy, Z - 0.0032]
    else:
        rospy.loginfo("picking")
        position = [HANGING_POSITION[0] + dx, HANGING_POSITION[1] + dy, Z]
    
    pose = rtde_help.getPoseObj(position, set_orientation)
    
    if place:
        pose.pose.position.z = 0.02
        rtde_help.goToPose(pose)
        msg = PushPull()
        msg.state = PUSH_STATE
        msg.pwm = DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(1)
    else:
        pose.pose.position.z = 0.02

        rtde_help.goToPose(pose)
        pose.pose.position.z = Z

        rtde_help.goToPose(pose)
        haptic_search()

    pose.pose.position.z = 0.100
    rtde_help.goToPose(pose)
    msg = PushPull()
    msg.state = 0
    msg.pwm = DUTYCYCLE_100
    PushPull_pub.publish(msg)

def get_graveyard_position(piece, is_white):
    """
    Returns the position of the piece in the graveyard.
    """
    file = PIECE_TO_FILE[piece.lower()]
    rank = 9 if is_white else 10
    return FILES[file], RANKS[rank]

def handle_capture(pickup_x, pickup_y, place_x, place_y, piece):
    """
    Handles moving a captured piece to the graveyard.
    """
    rospy.loginfo("capturing")
    gx, gy = get_graveyard_position(piece, piece.isupper())
    rospy.loginfo(f"graveyard position: {gx}, {gy}")
    pick_and_place(place_x, place_y, gx, gy)
    pick_and_place(pickup_x, pickup_y, place_x, place_y)
    rospy.loginfo("captured")

def handle_en_passant(pickup_x, pickup_y, place_x, place_y, move):
    """
    Handles en passant by moving the captured pawn to the graveyard.
    """
    rospy.loginfo("en passant")
    sp, ep = move[:2], move[2:]
    captured_pawn_file = ep[0]
    captured_pawn_rank = int(ep[1]) - 1
    # gx, gy = get_graveyard_position(piece, piece.isupper())

    # pick_and_place(place_x, place_y, gx, gy)


def handle_castling(move):
    """
    Handles castling by moving the rook to its new position.
    """
    rospy.loginfo("castling")
    if move in ["e1g1", "e8g8"]:  # Kingside castling
        rook_start = "h1" if move == "e1g1" else "h8"
        rook_end = "f1" if move == "e1g1" else "f8"
    elif move in ["e1c1", "e8c8"]:  # Queenside castling
        rook_start = "a1" if move == "e1c1" else "a8"
        rook_end = "d1" if move == "e1c1" else "d8"
    else:
        return

    rook_sp_x, rook_sp_y, rook_ep_x, rook_ep_y = get_offsets(rook_start, rook_end)
    king_sp_x, king_sp_y, king_ep_x, king_ep_y = get_offsets(move[:2], move[2:])

    pick_and_place(king_sp_x, king_sp_y, king_ep_x, king_ep_y) # pick and place king
    rospy.loginfo("king moved")
    pick_and_place(rook_sp_x, rook_sp_y, rook_ep_x, rook_ep_y) # pick and place rook
    rospy.loginfo("rook moved")

def handle_pawn_promotion(pickup_x, pickup_y, place_x, place_y, piece, captured_piece):
    """
    Handles pawn promotion by replacing the pawn with a piece from the graveyard.
    """
    rospy.loginfo("promoting")
    is_white = piece.isupper()
    if captured_piece:
        # graveyard position of the captured piece
        gx, gy = get_graveyard_position(captured_piece, captured_piece.isupper())

        handle_capture(pickup_x, pickup_y, place_x, place_y, captured_piece)
        # graveyard position for the pawn
        gx_pawn, gy_pawn = get_graveyard_position(piece, piece.isupper())
        print(gx_pawn, gy_pawn)
        pick_and_place(place_x, place_x, gx_pawn, gy_pawn)  
    else:
        gx, gy = get_graveyard_position(piece, piece.isupper())

        pick_and_place(pickup_x, pickup_y, gx, gy)
    
    if is_white:
        pick_and_place(GULAG_WHITE[0], GULAG_WHITE[1], place_x, place_y, picking_up_promotion=True)
    else:
        pick_and_place(GULAG_BLACK[0], GULAG_BLACK[1], place_x, place_y, picking_up_promotion=True)

    
def get_offsets(sp, ep):
    """
    Returns pickup_x, pickup_y, place_x, place_y offsets needed to move from the starting position to the ending position.
    
    """
    return FILES[sp[0]], RANKS[int(sp[1])], FILES[ep[0]], RANKS[int(ep[1])]

def reset_board():
    """
    Resets the chess board to its initial state.
    Graveyard state
    ------------
    |p|n|b|q|r|
    |P|N|B|Q|R|
    ------------
    """
    rospy.loginfo("resetting board")
    
    move_to_hanging_position()
    rospy.loginfo("board reset")


# def handshake():
#     """
#     Shuts down the node after a checkmate.
#     """
#     handshake_orientation = tf.transformations.quaternion_from_euler(np.pi - 75*deg2rad, 0 , -np.pi/8,'sxyz')
#     handshake_pose = rtde_help.getPoseObj(rtde_help.get, [shooting_orientation[0], shooting_orientation[1], shooting_orientation[2], shooting_orientation[3]])

#     rtde_help.goToPose(hand)

def make_move(message):
    """
    Moves the chess piece to the specified location on the board.
    """
    move_to_hanging_position()
    # if message.is_checkmate:
    #     handshake()
    #     rospy.signal_shutdown("Checkmate")

    move = message.chess_move
    piece = message.piece_to_move
    captured_piece = message.captured_piece
    is_piece_white = piece.isupper()
    sp, ep = move[:2], move[2:]
    pickup_x, pickup_y, place_x, place_y = get_offsets(sp, ep)
    is_promotion = piece in ("p", "P") and (ep[1] == "8" or ep[1] == "1")
    if message.is_capture != "no capture" and not is_promotion:  
        if message.is_capture == "en passant":
            handle_en_passant(pickup_x, pickup_y, place_x, place_y, move)
        else:
            handle_capture(pickup_x, pickup_y, place_x, place_y, captured_piece)
    elif piece.lower() == "k" and abs(ord(sp[0]) - ord(ep[0])) > 1:
        handle_castling(move)
    elif is_promotion:
        handle_pawn_promotion(pickup_x, pickup_y, place_x, place_y, piece, captured_piece)
    else:
        pick_and_place(pickup_x, pickup_y, place_x, place_y) 

    msg = PushPull()
    msg.state = 0
    msg.pwm = DUTYCYCLE_100
    PushPull_pub.publish(msg)

    move_to_hanging_position()
def pick_and_place(pickup_x, pickup_y, place_x, place_y, picking_up_promotion=False):
    pick_or_place(pickup_x, pickup_y, place=False, picking_up_promotion=picking_up_promotion)  
    (dx, dy) = (place_x - pickup_x, place_y - pickup_y)
    pick_or_place(dx, dy, place=True)

def blind_haptic_search(ch=4, reverse=False):

    FT_SimulatorOn = False
    np.set_rospy.loginfooptions(precision=4)

    # Setup helper functions
    FT_help = FT_CallbackHelp() # it deals with subscription.
    rospy.sleep(0.5)
    P_help = P_CallbackHelp() # it deals with subscription.

    rospy.sleep(0.5)
    file_help = fileSaveHelp()
    search_help = hapticSearch2DHelp(d_lat = 5e-3, d_yaw=1, n_ch = 4, p_reverse = False) # d_lat is important for the haptic search (if it is too small, the controller will fail)


    if FT_SimulatorOn:
        rospy.loginfo("wait for FT simul")
        rospy.wait_for_service('start_sim')
        # bring the service
        netftSimCall = rospy.ServiceProxy('start_sim', StartSim)

    # Set the PushPull Publisher  
    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
    rospy.sleep(0.5)
    msg = PushPull()
    msg.state, msg.pwm = 0, 0

    # Set the synchronization Publisher
    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)

    rospy.loginfo("Wait for the data_logger to be enabled")
    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
    dataLoggerEnable(False) # reset Data Logger just in case
    rospy.sleep(1)
    file_help.clearTmpFolder()        # clear the temporary folder
    datadir = file_help.ResultSavingDirectory
    

    # Set the disengage pose
    disengagePosition = [0.502, -0.200, 0.280]
    setOrientation = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r) #static (s) rotating (r)
    disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
    
    
    # set initial parameters
    suctionSuccessFlag = False
    # P_vac = search_help.P_vac
    timeLimit = 15
    pathLimit = 50e-3

    FT_SimulatorOn = False
    np.set_rospy.loginfooptions(precision=4)

    FT_help = FT_CallbackHelp()
    P_help = P_CallbackHelp() # it deals with subscription.

    
    if FT_SimulatorOn:
        rospy.wait_for_service('start_sim')
        netftSimCall = rospy.ServiceProxy('start_sim', StartSim)

    rospy.sleep(0.5)
    msg = PushPull()
    msg.state, msg.pwm = 0, 0

    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)

    suctionSuccessFlag = False

    rospy.loginfo("Start the haptic search")
    msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
    PushPull_pub.publish(msg)
    P_help.startSampling()
    rospy.sleep(0.5)
    P_help.setNowAsOffset()
    dataLoggerEnable(True)

    P_vac = 100

    while not suctionSuccessFlag:
        msg.state = PULL_STATE
        msg.pwm = DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(1)


        P_array = P_help.four_pressure
        rospy.loginfo(P_array)
        if all(np.array(P_array) < P_vac):
            suctionSuccessFlag = True
            rospy.loginfo("Suction engage succeeded")
            rospy.sleep(1)
            return
        else:
            position = [
                rtde_help.getCurrentPose().pose.position.x + random.random() * .01 - 0.005,
                rtde_help.getCurrentPose().pose.position.y + random.random() * .01 - 0.005,
                Z + .01
            ]            
            pose = rtde_help.getPoseObj(position, set_orientation)
            msg.state = PUSH_STATE
            msg.pwm = DUTYCYCLE_100
            PushPull_pub.publish(msg)
            rtde_help.goToPose(pose)

            position = [rtde_help.getCurrentPose().pose.position.x, rtde_help.getCurrentPose().pose.position.y , Z ]
            pose = rtde_help.getPoseObj(position, set_orientation)
            rtde_help.goToPose(pose)


def haptic_search(ch=4, reverse=False):

    FT_SimulatorOn = False
    np.set_printoptions(precision=4)
    file_help = fileSaveHelp()
    search_help = hapticSearch2DHelp(d_lat = 5e-3, d_yaw=1, n_ch = 4, p_reverse = False) # d_lat is important for the haptic search (if it is too small, the controller will fail)
    # Set the PushPull Publisher  
    PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
    rospy.sleep(0.25)
    msg = PushPull()
    msg.state, msg.pwm = 0, 0

    # Set the synchronization Publisher
    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)

    # rospy.loginfo("Wait for the data_logger to be enabled")
    # rospy.wait_for_service('data_logging')
    # dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
    # dataLoggerEnable(False) # reset Data Logger just in case
    # rospy.sleep(1)
    # file_help.clearTmpFolder()        # clear the temporary folder
    # datadir = file_help.ResultSavingDirectory
    
    
    # set initial parameters
    np.set_printoptions(precision=4)
    msg = PushPull()
    msg.state, msg.pwm = 0, 0

    syncPub = rospy.Publisher('sync', Int8, queue_size=1)
    syncPub.publish(SYNC_RESET)

    suctionSuccessFlag = False

    rospy.loginfo("Start the haptic search")
    msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
    # PushPull_pub.publish(msg)
    P_help.startSampling()
    rospy.sleep(0.25)
    P_help.setNowAsOffset()
    P_vac = -3500

    initial_pose = rtde_help.getCurrentPose().pose.position
    while not suctionSuccessFlag:
        msg.state = PULL_STATE
        msg.pwm = 100
        PushPull_pub.publish(msg)
        rospy.sleep(1)
        P_array = P_help.four_pressure
        if all(np.array(P_array) < P_vac):
            suctionSuccessFlag = True
            rospy.loginfo("Suction engage succeeded")
            return
        measuredCurrPose = rtde_help.getCurrentPose()
        T_curr = search_help.get_Tmat_from_Pose(measuredCurrPose)

        # calculate transformation matrices
        T_later, T_align = search_help.get_Tmats_from_controller(P_array)
        T_move =  T_later
        currPose = search_help.get_PoseStamped_from_T_initPose(T_move, measuredCurrPose)
        arr = search_help.get_Tmat_lateralMove(P_array)[:, -1]
        vector = search_help.get_lateral_direction_vector(P_array)
        norm = np.linalg.norm(vector)
        dx, dy = 0, 0
        if norm >= 0.1:
            normalized = vector / np.linalg.norm(vector)
            dx, dy = normalized[0] * 0.03, normalized[1] * 0.025
        print(dx, dy)
        if all(np.array(P_array) < P_vac):
            suctionSuccessFlag = True
            rospy.loginfo("Suction engage succeeded")
            return
        else:
            # go upwards to avoid collision
            position = [
                currPose.pose.position.x,
                currPose.pose.position.y,
                Z + 0.01
            ]
            pose = rtde_help.getPoseObj(position, set_orientation)
            rtde_help.goToPose(pose)

            # move to new pose adaptively
            position = [
                currPose.pose.position.x + dx,
                currPose.pose.position.y,
                Z
            ]
            pose = rtde_help.getPoseObj(position, set_orientation)
            rtde_help.goToPose(pose)

if __name__ == '__main__':
    try:
        rospy.init_node('suction_cup')
        rtde_help = rtdeHelp()
        rospy.sleep(0.5)
        set_orientation = tf.transformations.quaternion_from_euler(np.pi,0,np.pi,'sxyz')
        PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
        FT_help = FT_CallbackHelp() 
        P_help = P_CallbackHelp() 
        listener()
    except KeyboardInterrupt:
        PushPull_pub.publish(PushPull(state=0, pwm=0))
        pass