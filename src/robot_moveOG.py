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
    print("Couldn't import ROS. I assume you're running this on your laptop")
    ros_enabled = False

hanging_position = [0.567, -0.0623, 0.280]
Z = 0.0165

def listener():
    
    if ros_enabled:

        rospy.Subscriber('/move_to_make', ChessState, make_move)
        rospy.spin()

        
def move_to_hanging_position():
    """
    Moves the chess piece to the origin on the board.
    """
    origin = rtde_help.getPoseObj(hanging_position, set_orientation)
    rtde_help.goToPose(origin)

def move_to_graveyard():
    graveyard_position = [0.567, -0.0623, 0.0165]
    graveyard = rtde_help.getPoseObj(graveyard_position, set_orientation)
    rtde_help.goToPose(graveyard)


def pick_or_place(dx, dy, place=False):
    """
    Picks up the chess piece from the board.
    """
    if place:
        position = [rtde_help.getCurrentPose().pose.position.x + dx, rtde_help.getCurrentPose().pose.position.y + dy, Z]
    else:
        position = [hanging_position[0] + dx, hanging_position[1] + dy, Z]
        
    pose = rtde_help.getPoseObj(position, set_orientation)
    rtde_help.goToPose(pose)
    
    pose.pose.position.z = 0.100
    rtde_help.goToPose(pose)





def make_move(message):
    """
    Moves the chess piece to the specified location on the board.
    """
    move_to_hanging_position()

    cols = dict(zip(('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'), np.arange(-0.2667, 0.3429, 0.0762)))
    rows = dict(zip((1, 2, 3, 4, 5, 6, 7, 8), np.arange(-.2667, 0.3429, 0.0762)))

    sp, ep = message.chess_move[:2], message.chess_move[2:]
    pickup_y = rows[int(sp[1])]
    pickup_x = cols[sp[0]]
    place_y = rows[int(ep[1])]
    place_x = cols[ep[0]]

    pick_or_place(pickup_x, pickup_y)
    (dx, dy) = (place_x - pickup_x, place_y - pickup_y)
    pick_or_place(dx, dy, place=True)
    move_to_hanging_position()

if __name__ == '__main__':
    try:
        rospy.init_node('move_maker')
        rtde_help = rtdeHelp()
        rospy.sleep(0.5)
        set_orientation = tf.transformations.quaternion_from_euler(np.pi,0,np.pi,'sxyz')

        listener()
    except KeyboardInterrupt:
        pass