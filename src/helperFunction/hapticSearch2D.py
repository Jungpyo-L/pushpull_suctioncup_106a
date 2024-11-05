#!/usr/bin/env python

import os
import datetime
import numpy as np
import re
from geometry_msgs.msg import PoseStamped
from .utils import rotation_from_quaternion, create_transform_matrix, rotationFromQuaternion, normalize, hat, quaternionFromMatrix, quaternion_from_matrix
from scipy.spatial.transform import Rotation as Rot
import scipy
from icecream import ic


class hapticSearch2DHelp(object):
    def __init__(self,dP_threshold=10, dw=15, P_vac = -20000, d_lat = 1.5e-3, d_z= 1.5e-3, d_yaw = 0.3, n_ch = 4, p_reverse = False):
        # for first performance test dw=15, d_lat = 0.5e-2, d_z= 1.5e-3
        # original dw = 3, d_lat = 0.1e-2, d_z = 0.3e-3
        self.dP_threshold = dP_threshold
        self.dw = dw * np.pi / 180.0
        
        self.P_vac = P_vac
        self.p_reverse = p_reverse
        self.d_lat = d_lat
        self.d_z_normal = d_z
        self.d_yaw = d_yaw

        # for Brownian motion (initial condition)
        self.x0 = 0 
        self.BM_step = 0
        self.BM_x = 0
        self.BM_y = 0
        
        # for number of chambers of the suction cup
        self.n = n_ch
        
        # for momemtum controller
        self.velocity = np.array([0.0, 0.0]) # initial velocity
        self.d_lat_momentum = self.d_lat * 0.3 # initial step size for momentum controller
        self.damping_factor = 0.9
        self.max_velocity_x = self.d_lat * 1.5
        self.max_velocity_y = self.d_lat * 1.5
    
    def get_yawRotation_from_T(self, T):
        R = T[0:3,0:3]
        quat = quaternion_from_matrix(R)
        r = Rot.from_quat(quat)
        return r.as_euler('zyx')[0]

    def get_ObjectPoseStamped_from_T(self,T):   #transformation
        thisPose = PoseStamped()
        thisPose.header.frame_id = "base_link"
        R = T[0:3,0:3]
        quat = quaternion_from_matrix(R)   #original, quat matrix
        position = T[0:3,3]
        [thisPose.pose.position.x, thisPose.pose.position.y, thisPose.pose.position.z] = position
        [thisPose.pose.orientation.x, thisPose.pose.orientation.y, thisPose.pose.orientation.z,thisPose.pose.orientation.w] = quat
        return thisPose

    def get_Tmat_from_Pose(self,PoseStamped):  #format
        quat = [PoseStamped.pose.orientation.x, PoseStamped.pose.orientation.y, PoseStamped.pose.orientation.z, PoseStamped.pose.orientation.w]        
        translate = [PoseStamped.pose.position.x, PoseStamped.pose.position.y, PoseStamped.pose.position.z]
        return self.get_Tmat_from_PositionQuat(translate, quat)   # original
        # return translate +quat    
    
    def get_Tmat_from_PositionQuat(self, Position, Quat):    #transformation
        rotationMat = rotation_from_quaternion(Quat)   #original
        T = create_transform_matrix(rotationMat, Position)
        return T

    def get_PoseStamped_from_T_initPose(self, T, initPoseStamped):   #transformation
        T_now = self.get_Tmat_from_Pose(initPoseStamped)    #original
        targetPose = self.get_ObjectPoseStamped_from_T(np.matmul(T_now, T))   #original
        # targetPose = self.get_ObjectPoseStamped_from_T(T)   #rtde
        return targetPose

    def get_Tmat_TranlateInBodyF(self, translate = [0., 0., 0.]): #format
        return create_transform_matrix(np.eye(3), translate)

    def get_Tmat_TranlateInZ(self, direction = 1):     #format
        offset = [0.0, 0.0, np.sign(direction)*self.d_z_normal]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate = offset)

    def get_Tmat_TranlateInY(self, direction = 1):
        offset = [0.0, np.sign(direction)*self.d_lat, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate = offset)
    
    def get_Tmat_TranlateInX(self, direction = 1):
        offset = [np.sign(direction)*self.d_lat, 0.0, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate = offset)
    
    # Sould set the first chamber of the suction cup is in direction of the positive x-axis
    # Keep in mind that yaw angle doen't need to be considered in this function because it is about body frame
    def calculate_unit_vectors(self, num_chambers, yaw_angle):
        return [np.array([np.cos(2 * np.pi / (num_chambers * 2) + 2 * np.pi * i / num_chambers),
                      np.sin(2 * np.pi / (num_chambers * 2) + 2 * np.pi * i / num_chambers)])
            for i in range(num_chambers)]

    def calculate_direction_vector(self, unit_vectors, vacuum_pressures):
        direction_vector = np.sum([vp * uv for vp, uv in zip(vacuum_pressures, unit_vectors)], axis=0)
        return direction_vector / np.linalg.norm(direction_vector) if np.linalg.norm(direction_vector) > 0 else np.array([0, 0])
    
    def get_lateral_direction_vector(self, P_array, yaw_angle, thereshold = True):
        if thereshold:
            th = self.dP_threshold
        else:
            th = 0
        # make the pressure array positive (vacuum pressure)
        if not self.p_reverse:
            P_array = [-P for P in P_array]
        # check if the invididual vacuum pressure is above the threshold
        # if not, then set the pressure to zero
        P_array = [P if P > th else 0 for P in P_array]
        unit_vectors = self.calculate_unit_vectors(self.n, yaw_angle)
        return self.calculate_direction_vector(unit_vectors, P_array)
        
    def get_Tmat_lateralMove(self, P_array, yaw_angle):
        v = self.get_lateral_direction_vector(P_array, yaw_angle, True)
        v_step = v * self.d_lat
        # caution: the x and y axis are swapped in the coordinate system
        # positive x-axis is towards south
        # positive y-axis is towards west
        # positive z-axis is towards down
        return self.get_Tmat_TranlateInBodyF([-v_step[1], -v_step[0], 0.0])
    
    def get_Tmat_lateralMoveOld(self, P_array, yaw_angle, weightVal = 1.0):
        d_lat = self.d_lat
        dP_threshold = self.dP_threshold

        P1, P2, P3, P0 = P_array

        PW = (P3 + P2)/2
        PE = (P1 + P0)/2
        PN = (P1 + P2)/2
        PS = (P0 + P3)/2

        # pressure differentials
        dP_WE = PW - PE        # 0 deg
        dP_SN = PS - PN        # 90 deg
        dP_NW_SE = P2 - P0     # 45 deg
        dP_SW_NE = P3 - P1     # -45 deg

        dx_lat = 0.0
        dy_lat = 0.0

        # added by Jungpyo (220909)
        r_p = np.zeros(2)
        r_p[0] = -dP_WE
        r_p[1] = -dP_SN
        r_p = r_p / np.linalg.norm(r_p)

        if abs(dP_WE) > dP_threshold:
            dy_lat = r_p[0] * d_lat * weightVal
            
        if abs(dP_SN) > dP_threshold:
            dx_lat = r_p[1] * d_lat * weightVal

        T = self.get_Tmat_TranlateInBodyF([dx_lat, dy_lat, 0.0])
        return T
    
    # get Tmat for momentum controller
    def get_Tmat_momentumMove(self, P_array, yaw_angle):
        v = self.get_lateral_direction_vector(P_array, yaw_angle, True)
        self.velocity = self.damping_factor * self.velocity + v * self.d_lat
        # limit the velocity
        self.velocity[0] = np.clip(self.velocity[0], -self.max_velocity_x, self.max_velocity_x)
        self.velocity[1] = np.clip(self.velocity[1], -self.max_velocity_y, self.max_velocity_y)
        # caution: the x and y axis are swapped in the coordinate system
        # positive x-axis is towards south
        # positive y-axis is towards west
        # positive z-axis is towards down
        return self.get_Tmat_TranlateInBodyF([-self.velocity[1], -self.velocity[0], 0.0])
    
    def get_Tmat_yawRotation(self):
        d_yaw = self.d_yaw * np.pi/180
        # rotation axis to the z-direction
        # positive z-axis is towards down
        rot_axis = np.array([0,0,-1])
        omega_hat = hat(rot_axis)
        Rw = scipy.linalg.expm(d_yaw * omega_hat)   
        return create_transform_matrix(Rw, [0,0,0])
        
    def get_Tmats_from_controller(self, P_array, yaw_angle, controller_str = "normal"):
        # ["normal","yaw","momentum","momentum_yaw"]
        if controller_str == "normal":
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove(P_array, yaw_angle)
            T_yaw = np.eye(4)
        
        elif controller_str == "yaw":
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove(P_array, yaw_angle)
            T_yaw = self.get_Tmat_yawRotation()
        
        elif controller_str == "momentum":
            T_align = np.eye(4)
            T_later = self.get_Tmat_momentumMove(P_array, yaw_angle)
            T_yaw = np.eye(4)

        elif controller_str == "momentum_yaw":
            T_align = np.eye(4)
            T_later = self.get_Tmat_momentumMove(P_array, yaw_angle)
            T_yaw = self.get_Tmat_yawRotation()

        return T_later, T_yaw, T_align
    

    def get_Tmat_lateralMove_random(self):
        d_lat = self.d_lat
        theta = np.random.rand() * 2*np.pi
        dx_lat = d_lat * np.cos(theta)
        dy_lat = d_lat * np.sin(theta)

        T = self.get_Tmat_TranlateInBodyF([dx_lat, dy_lat, 0.0])
        return T
    
    def get_Tmat_lateralMove_BM(self):
        
        dx_lat = (self.BM_x[self.BM_step + 1] - self.BM_x[self.BM_step])
        dy_lat = (self.BM_y[self.BM_step + 1] - self.BM_y[self.BM_step])

        T = self.get_Tmat_TranlateInBodyF([dx_lat, dy_lat, 0.0])
        return T

    def get_BM(self, n_step= 125*15, sigma = 0.03):
        w = np.ones(n_step)*self.x0

        for i in range(1,n_step):
            # Sampling from the NOrmal distribution
            yi = np.random.normal()
            # Weiner process
            w[i] = w[i-1]+(yi/np.sqrt(n_step))*sigma
        
        return w

    def get_Tmats_Suction(self, weightVal):
        P_array = [20,0,0,20]
        T_align = self.get_Tmat_alignSuction(P_array, weightVal=weightVal )
        T_later = self.get_Tmat_lateralMove(P_array, weightVal=1.0-weightVal)
        return T_later, T_align
    
    def get_Tmats_alignSuctionLateralMode(self, P_array, weightVal):
        T_align = self.get_Tmat_alignSuction(P_array, weightVal=weightVal )
        T_later = self.get_Tmat_lateralMove(P_array, weightVal=1.0-weightVal)
        return T_later, T_align
        

    def get_Tmat_axialMove(self, F_normal, F_normalThres):
        
        if F_normal > -F_normalThres[0]:
            # print("should be pushing towards surface in cup z-dir")
            T_normalMove = self.get_Tmat_TranlateInZ(direction = 1)
        elif F_normal < -F_normalThres[1]:
            # print("should be pulling away from surface in cup z-dir")
            T_normalMove = self.get_Tmat_TranlateInZ(direction=-1)
        else:
            T_normalMove = np.eye(4)
        return T_normalMove

    def intersection_between_cup_box(self, T_targetPose, BoxTopLeftCorner_meter, BoxBottomRightCorner_meter):
        positionVec = T_targetPose[3,0:3]
        rayVec = -T_targetPose[2,0:3] # z axis is aligned to downward suction cup axis.
        rayCoefficients = np.zeros(4)
        rayCoefficients[0] = (BoxTopLeftCorner_meter[0]-positionVec[0])/rayVec[0]
        rayCoefficients[1] = (BoxTopLeftCorner_meter[1]-positionVec[1])/rayVec[1]
        rayCoefficients[2] = (BoxBottomRightCorner_meter[0]-positionVec[0])/rayVec[0]
        rayCoefficients[3] = (BoxBottomRightCorner_meter[1]-positionVec[1])/rayVec[1]

        closestCoef = np.amin(rayCoefficients[rayCoefficients>0])
        intersecPoint = positionVec + closestCoef*rayVec
        return intersecPoint


    def getGoalPosestampedFromGQCNN(self, T, thisPose, initEndEffPosestamped ):
        #From pose Information of GQCNN, get the transformed orientation of the UR10
        
        initEndEffectorPosition = [initEndEffPosestamped.pose.position.x, initEndEffPosestamped.pose.position.y, initEndEffPosestamped.pose.position.z]
        initEndEffectorQuat = [initEndEffPosestamped.pose.orientation.x, initEndEffPosestamped.pose.orientation.y, initEndEffPosestamped.pose.orientation.z, initEndEffPosestamped.pose.orientation.w]
        
        deltaPosition = np.matmul( T, (np.array([thisPose.position.x, thisPose.position.y, thisPose.position.z, 1])))  
        goalRobotPosition = deltaPosition[0:3] + np.array(initEndEffectorPosition)
        
        # orient
        thisQuat = [thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z, thisPose.orientation.w]

        r_pose_from_cam = Rot.from_quat(thisQuat)
        axis_in_cam = r_pose_from_cam.as_matrix()
        # print(axis_in_cam)
        targetVec = axis_in_cam[:,0] # rotated x is the target vector
        
        R_N_cam = T[0:3,0:3]
        targetSuctionAxisVec_N = np.matmul(R_N_cam,targetVec)
        # print(targetSuctionAxisVec_N)

        # to us, z axis of the the tool0 should be the 

        r_currOrient_RobotEff = Rot.from_quat(initEndEffectorQuat)
        currSuctionAxisVec_N = r_currOrient_RobotEff.as_matrix()[:,2]

        rotAxis = np.cross(currSuctionAxisVec_N, targetSuctionAxisVec_N)  
        rotAxis /= np.linalg.norm(rotAxis)
        angleBtwTwo = np.arccos(np.dot(currSuctionAxisVec_N, targetSuctionAxisVec_N))

        rotAxis_in_BodyF = r_currOrient_RobotEff.apply(rotAxis, inverse=True)        
        r_RotOrient = Rot.from_mrp(rotAxis_in_BodyF * np.tan(angleBtwTwo / 4))
        
        r_targetOrient_RobotEff = r_currOrient_RobotEff*r_RotOrient
        
        targetOrient_quat = r_targetOrient_RobotEff.as_quat()

        T_pose = self.get_Tmat_from_PositionQuat(goalRobotPosition,targetOrient_quat)
        return self.get_ObjectPoseStamped_from_T(T_pose)
        # return goalRobotPosition, targetOrient_quat, targetSuctionAxisVec_N  

    def getGoalPosestampedFromCam(self, T, thisPose, initEndEffPosestamped):
        #From pose Information of GQCNN, get the transformed orientation of the UR10
        
        initEndEffectorPosition = [initEndEffPosestamped.pose.position.x, initEndEffPosestamped.pose.position.y, initEndEffPosestamped.pose.position.z]
        initEndEffectorQuat = [initEndEffPosestamped.pose.orientation.x, initEndEffPosestamped.pose.orientation.y, initEndEffPosestamped.pose.orientation.z, initEndEffPosestamped.pose.orientation.w]

        targetPosition = np.array([thisPose[0], thisPose[1], thisPose[2], 1])
        goalRobotPosition = np.matmul(T, targetPosition)
        goalRobotPosition[0:3] = goalRobotPosition[0:3] + np.array(initEndEffectorPosition)

        T_pose = self.get_Tmat_from_PositionQuat(goalRobotPosition[0:3], np.array(initEndEffectorQuat))
        return self.get_ObjectPoseStamped_from_T(T_pose)

