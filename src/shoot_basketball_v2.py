#!/usr/bin/env python

try:
    import rospy
    import tf
    from geometry_msgs.msg import Point
    ros_enabled = True
except:
    print("Couldn't import ROS. I assume you're running this on your laptop")
    ros_enabled = False

from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat
import numpy as np
from helperFunction.rtde_helper import rtdeHelp
from pushpull_suctioncup_106a.msg import PushPull



class BasketballShooter:
    def __init__(self):
        self.goal_point = None  # Store the received goal point
    
        if ros_enabled:

            # Subscribe to the 'goal_point' topic
            self.goal_subscriber = rospy.Subscriber(
                'goal_point', Point, self.goal_point_callback
            )
            rospy.loginfo("Subscribed to 'goal_point' topic")

    def goal_point_callback(self, msg):
        """
        Callback function to handle incoming messages from 'goal_point' topic.
        """
        self.goal_point = [msg.x, msg.y, msg.z]
        rospy.loginfo(f"Received goal point: {self.goal_point}")

    def align_with_hoop(self, rtde_help):
        """
        Aligns the robot arm with the x position of the hoop.
        """
        if self.goal_point is None:
            rospy.logerr("Goal point is not available. Cannot align with hoop.")
            return

        current_pose = rtde_help.getCurrentPose()
        rospy.loginfo(f"Current Pose: {current_pose}")

        aligned_position = current_pose[:3]
        aligned_position[0] = self.goal_point[0]  # align x position with hoop

        # keep orientation the same as current pose
        orientation = current_pose[3:]

        # new pose
        aligned_pose = rtde_help.getPoseObj(aligned_position, orientation)
        rospy.loginfo(f"Aligning with the hoop at position: {aligned_position}")

        # move there
        try:
            input("Press <Enter> to move to the aligned position")
            rtde_help.goToPose(aligned_pose)
            rospy.loginfo("Robot arm aligned with the hoop's x position.")
        except rospy.ROSInterruptException:
            rospy.logerr("ROS Interrupt Exception occurred.")
            return
        except KeyboardInterrupt:
            rospy.logwarn("Process interrupted by user.")
            return

    def rotate_arm_x(self, rtde_help, angular_velocity, duration):
        """
        Rotates the robot arm along the x-axis at a specified angular velocity.

        Args:
        - rtde_help: Instance of the RTDE helper class.
        - angular_velocity: Angular velocity in radians/second.
        - duration: Duration for which the rotation should be applied (in seconds).
        """
        rospy.loginfo(f"Rotating arm along x-axis at {angular_velocity} rad/s for {duration} seconds.")

        current_pose = rtde_help.getCurrentPose()
        orientation_quat = current_pose[3:]

        rotation_matrix = tf.transformations.quaternion_matrix(orientation_quat)

        angular_velocity_vector = np.array([angular_velocity, 0, 0])  # rotation around x-axis

        delta_rotation = np.eye(4) 
        delta_rotation[:3, :3] = tf.transformations.rotation_matrix(angular_velocity * duration, [1, 0, 0])[:3, :3]

        new_rotation_matrix = np.dot(rotation_matrix, delta_rotation[:3, :3])

        new_orientation_quat = tf.transformations.quaternion_from_matrix(new_rotation_matrix)

        new_pose = rtde_help.getPoseObj(current_pose[:3], new_orientation_quat)

        try:
            rospy.loginfo(f"Rotating to new pose: {new_pose}")
            rtde_help.goToPose(new_pose)
            rospy.loginfo("Rotation complete.")
        except rospy.ROSInterruptException:
            rospy.logerr("ROS Interrupt Exception occurred during rotation.")
            return
        except KeyboardInterrupt:
            rospy.logwarn("Process interrupted by user.")
            return

    def release_ball_at_angle(self, rtde_help, target_angle_deg=30):
        """
        Releases the ball when the arm reaches the specified angle.

        Args:
        - rtde_help: Instance of the RTDE helper class.
        - target_angle_deg: Target release angle in degrees.
        """
        rospy.loginfo(f"Releasing the ball at {target_angle_deg} degrees.")

        target_angle_rad = np.radians(target_angle_deg)

        rate = rospy.Rate(100)  # 100 Hz loop to monitor
        while not rospy.is_shutdown():
            current_pose = rtde_help.getCurrentPose()
            position = current_pose[:3]
            z_vector = np.array([0, 0, -1])  # vector straight down
            arm_vector = np.array(position) - np.array([0, 0, 0])  # arm vector relative to base

            z_vector = z_vector / np.linalg.norm(z_vector)
            arm_vector = arm_vector / np.linalg.norm(arm_vector)

            dot_product = np.dot(z_vector, arm_vector)
            current_angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0)) # clamp

            rospy.loginfo_throttle(0.1, f"Current arm angle: {np.degrees(current_angle_rad)} degrees")

            if current_angle_rad >= target_angle_rad:
                rospy.loginfo("Target angle reached. Releasing the ball.")
                self.release_suction()
                break

            rate.sleep()

    def shoot_basketball(self, rtde_help):
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
        positionA = [0.502, -0.200, 0.280]
        orientationA = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2, 'sxyz')  # static (s) rotating (r)
        poseA = rtde_help.getPoseObj(positionA, orientationA)
        try:
            input("Press <Enter> to go to pose A")
            rtde_help.goToPose(poseA)
            print("============ Return Home ============")
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

        # wait for the goal point
        rospy.loginfo("Waiting for goal point...")
        rate = rospy.Rate(10)  # 10 Hz
        while self.goal_point is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Still waiting for goal point...")
            rate.sleep()
        if self.goal_point:
            rospy.loginfo(f"Goal point received: {self.goal_point}")
        else:
            rospy.logwarn("No goal point received. Exiting.")
            return
        
        # align the x axis with the hoop such that the z axis of the hoop frame
        # and the robot frame are the same
        self.align_with_hoop(rtde_help)

        # rotate along x axis at whatever speed we want and for however long we want. 
        # really doesn't need to be percise as long as it's fast enough
        angular_velocity = 0.5 
        duration = 2.0  
        self.rotate_arm_x(rtde_help, angular_velocity, duration)

        # at 30 degrees, we want to release the pressure of the suction cup to release the ball
        self.release_ball_at_angle(rtde_help, 30)
        PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
        msg = PushPull()
        msg.state = 1
        msg.pwm = 100
        PushPull_pub.publish(msg)
    

if __name__ == '__main__':
    try:    
        print("test")
        rospy.init_node('shoot_basketball_v2')
        shooter = BasketballShooter()
        print("test1")
        if ros_enabled:
            
            print("test2")
            rtde_help = rtdeHelp()  # i have no idea what an RTDE helper is but i assume it's important
            shooter.shoot_basketball(rtde_help)
        else:
            print("ROS not enabled. Cannot proceed.")
    except rospy.ROSInterruptException:
        pass