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

#!/usr/bin/env python

try:
    import rospy
    import tf
    from geometry_msgs.msg import Point
    import copy
    

    ros_enabled = True
except:
    print("Couldn't import ROS. I assume you're running this on your laptop")
    ros_enabled = False

from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat
import numpy as np
from helperFunction.rtde_helper import rtdeHelp
deg2rad = np.pi / 180.0

class BasketballShooter:
    def __init__(self):
        self.goal_point = None  # Store the received goal point
    
        if ros_enabled:
            # Initialize ROS node
            rospy.init_node('basketball_shooter', anonymous=True)

            # Subscribe to the 'goal_point' topic
            # self.goal_subscriber = rospy.Subscriber(
            #     'goal_point', Point, self.goal_point_callback
            # )
            # rospy.loginfo("Subscribed to 'goal_point' topic")

    # def goal_point_callback(self, msg):
    #     """
    #     Callback function to handle incoming messages from 'goal_point' topic.
    #     """
    #     self.goal_point = [msg.x, msg.y, msg.z]
    #     rospy.loginfo(f"Received goal point: {self.goal_point}")

    # def align_with_hoop(self, rtde_help):
    #     """
    #     Aligns the robot arm with the x position of the hoop.
    #     """
    #     if self.goal_point is None:
    #         rospy.logerr("Goal point is not available. Cannot align with hoop.")
    #         return

    #     current_pose = rtde_help.getCurrentPose()
    #     rospy.loginfo(f"Current Pose: {current_pose}")

    #     aligned_position = current_pose[:3]
    #     aligned_position[0] = self.goal_point[0]  # align x position with hoop

    #     # keep orientation the same as current pose
    #     orientation = current_pose[3:]

    #     # new pose
    #     aligned_pose = rtde_help.getPoseObj(aligned_position, orientation)
    #     rospy.loginfo(f"Aligning with the hoop at position: {aligned_position}")

    #     # move there
    #     try:
    #         input("Press <Enter> to move to the aligned position")
    #         rtde_help.goToPose(aligned_pose)
    #         rospy.loginfo("Robot arm aligned with the hoop's x position.")
    #     except rospy.ROSInterruptException:
    #         rospy.logerr("ROS Interrupt Exception occurred.")
    #         return
    #     except KeyboardInterrupt:
    #         rospy.logwarn("Process interrupted by user.")
    #         return
    
    def rotate_joint_y_velocity(self, rtde_help):
        """
        Rotates a specific joint at a specified angular velocity for a given duration.

        Args:
        - rtde_help: Instance of the RTDE helper class.
        - joint_index: Index of the joint to move (0-based).
        - angular_velocity: Angular velocity in radians/second.
        - duration: Duration for which the rotation should be applied (in seconds).
        """
        current_joint_angles = rtde_help.rtde_r.getActualQ()
        print("Current joint angles: ", current_joint_angles)
        next_joint_angles = copy.deepcopy(current_joint_angles)
        next_joint_angles[4] = 3.14
        print("next joint angles: ", next_joint_angles)
        input("Press <Enter> to go to throw the ball")
        
        try:
            # shooting_orientation = tf.transformations.quaternion_from_euler(np.pi - 60*deg2rad, 0, 0,'sxyz')

            # shooting_pose = rtde_help.getPoseObj(rtde_help.getCurrentPose().pose.position, [shooting_orientation[0], shooting_orientation[1], shooting_orientation[2], shooting_orientation[3]])

            rtde_help.rtde_c.moveJ(next_joint_angles, 3.14, 10.0, False)
            # rtde_help.rtde_c.goToPoseAdaptive(shooting_pose)
            rospy.loginfo("Joint rotation complete.")
            rospy.sleep(2)
            next_joint_angles[4] = -np.pi/2
            rtde_help.rtde_c.moveJ(next_joint_angles, 3.14, 10.0, False)

        except rospy.ROSInterruptException:
            rospy.logerr("ROS Interrupt Exception occurred during joint velocity control.")
        except KeyboardInterrupt:
            rospy.logwarn("Process interrupted by user.")

    def shoot_basketball(self, rtde_help):        
        # pick up the ball in position A
        np.set_printoptions(precision=4)
        positionA = [0.502, -0.1, 0.280]
        orientationA = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2, 'sxyz')  # static (s) rotating (r)
        poseA = rtde_help.getPoseObj(positionA, orientationA)
        try:
            input("Press <Enter> to go to initial pose")
            rtde_help.goToPose(poseA)
            input("Press <Enter> to confirm that you have sucked the ball")
            PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
            rospy.sleep(0.5)
            msg = PushPull()
            msg.pwm = 100
            msg.state = 2
            PushPull_pub.publish(msg)


            print("============ Return Home ============")

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

        # wait for the goal point
        # rospy.loginfo("Waiting for goal point...")
        # rate = rospy.Rate(10)  # 10 Hz
        # while self.goal_point is None and not rospy.is_shutdown():
        #     rospy.loginfo_throttle(1, "Still waiting for goal point...")
        #     rate.sleep()
        # if self.goal_point:
        #     rospy.loginfo(f"Goal point received: {self.goal_point}")
        # else:
        #     rospy.logwarn("No goal point received. Exiting.")
        #     return
        
        #align the x axis with the hoop such that the z axis of the hoop frame
        #and the robot frame are the same
        #self.align_with_hoop(rtde_help)

        #rotate along x axis at whatever speed we want and for however long we want. 
        #really doesn't need to be percise as long as it's fast enough
        angular_velocity = 1.0 
        duration = 2.0  
        self.rotate_joint_y_velocity(rtde_help)
        
    

if __name__ == '__main__':
    try:
        shooter = BasketballShooter()
        if ros_enabled:
            rtde_help = rtdeHelp()  # i have no idea what an RTDE helper is but i assume it's important
            shooter.shoot_basketball(rtde_help)
        else:
            print("ROS not enabled. Cannot proceed.")
    except rospy.ROSInterruptException:
        pass