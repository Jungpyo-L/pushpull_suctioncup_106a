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
from pushpull_suctioncup_106a.msg import PushPull
from helperFunction.rtde_helper import rtdeHelp
from sensor_msgs.msg import JointState
#!/usr/bin/env python

try:
    import rospy
    import tf
    from geometry_msgs.msg import Point
    import rtde_receive

    ros_enabled = True
except:
    print("Couldn't import ROS. I assume you're running this on your laptop")
    ros_enabled = False

from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat
import numpy as np
from helperFunction.rtde_helper import rtdeHelp


def listener():
    if ros_enabled:
        rospy.Subscriber('/joint_states', JointState, release_ball_at_angle)

        rospy.spin()

def release_ball_at_angle(message):
    """
    Releases the ball when the arm reaches the specified angle.

    Args:
    - rtde_help: Instance of the RTDE helper class.
    - target_angle_deg: Target release angle in degrees.
    """

    target_angle_rad = 1.3
    current_angle_rad = message.position[4]
    print(f"Current angle: {current_angle_rad}")
    if current_angle_rad >= target_angle_rad:
        rospy.loginfo(f"Target angle reached. Releasing the ball. Target: {target_angle_rad}, Current: {current_angle_rad}")

        msg = PushPull()
        msg.state = 1
        msg.pwm = 100
        PushPull_pub.publish(msg)
        
   

if __name__ == '__main__':
    try:
        PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)

        rospy.init_node('BallGargler', anonymous=True)
        rospy.sleep(0.5)
        listener()
    except rospy.ROSInterruptException:
        pass