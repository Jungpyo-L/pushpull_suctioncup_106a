#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from helperFunction.rtde_helper import rtdeHelp

PLOTS_DIR = os.path.join(os.getcwd(), 'plots')




class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.image_pub = rospy.Publisher('detected_hoop', Image, queue_size=10)

        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message (look this message type up online)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def process_images(self):

        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)
        # TODO: Define range for cup color in HSV
        # Run `python hsv_color_thresholder.py` and tune the bounds so you only see your cup
        # update lower_hsv and upper_hsv directly
        #rtde_help = rtdeHelp(125)
        
        #rospy.sleep(0.5)
        #print(rtde_help.getCurrentPose())
        lower_hsv = np.array([96, 102, 52]) # TODO: Define lower HSV values for cup color
        upper_hsv = np.array([116, 230, 126]) # TODO: Define upper HSV values for cup color

        # TODO: Threshold the image to get only cup colors
        # HINT: Lookup cv2.inRange()
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # TODO: Get the coordinates of the cup points on the mask
        # HINT: Lookup np.nonzero()
        y_coords, x_coords = np.nonzero(mask)
        
        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
            print("No points detected. Is your color filter wrong?")
            return

        # Calculate the center of the detected region by 
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))

        # Fetch the depth value at the center
        depth = self.cv_depth_image[center_y, center_x]

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
            print("camera_x, camera_y, camera_z: ", camera_x / 1000, camera_y / 1000, camera_z / 1000)
            #camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            camera_link_x, camera_link_y, camera_link_z = camera_x, camera_y, camera_z
            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000
            print("Real-world coordinates in camera_link frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(camera_link_x, camera_link_y, camera_link_z))
            # Convert the (X, Y, Z) coordinates from camera frame to base (world) frame
            try:
                self.tf_listener.waitForTransform("/base", "/camera_link", rospy.Time(), rospy.Duration(10.0))
                point_base = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                X_base, Y_base, Z_base = point_base.point.x, point_base.point.y, point_base.point.z
                print("Real-world coordinates in base frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_base, Y_base, Z_base))

                if X_base < 0.001 and X_base > -0.001:
                    print("Erroneous goal point, not publishing - Is the cup too close to the camera?")
                else:
                    print("Publishing goal point: ", X_base, Y_base, Z_base)
                    # Publish the transformed point
                    self.point_pub.publish(Point(X_base, Y_base, Z_base))

                    # Overlay cup points on color image for visualization
                    cup_img = self.cv_color_image.copy()
                    cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
                    cv2.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)  # Draw green circle at center
                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                    self.image_pub.publish(ros_image)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return

if __name__ == '__main__':
    # deg2rad = np.pi / 180.0
    # np.set_printoptions(precision=4)
    # rtde_help = rtdeHelp(125)
    # rospy.sleep(0.5)
    # #orientationA = tf.transformations.quaternion_from_euler(np.pi,0,np.pi,'sxyz') #static (s) rotating (r)
    # orientationA = [-0.0045187, -0.73751, -0.67528637, 0.0068338]
    # #print(orientationA)
    # positionA = [0.5619, 0.1668, 0.4203]
    # poseA = rtde_help.getPoseObj(positionA, orientationA)


    # input("Press <Enter> to go to detection pose")
    # rtde_help.goToPose(poseA)
    ObjectDetector()

    