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
    def __init__(self,dP_threshold=10, dw=15, P_vac = -20000, d_lat = 0.5e-3, d_z= 1.5e-3, d_yaw = 0.3, n_ch = 4, p_reverse = False):
        # for first performance test dw=15, d_lat = 0.5e-2, d_z= 1.5e-3
        self.dP_threshold = dP_threshold
        self.dw = dw * np.pi / 180.0
        
        self.P_vac = P_vac
        self.p_reverse = p_reverse
        self.d_lat = d_lat
        self.d_z_normal = d_z
        self.d_yaw = d_yaw

        # for number of chambers of the suction cup
        self.n = n_ch
        

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

    def get_Tmat_TranslateInBodyF(self, translate = [0., 0., 0.]): #format
        return create_transform_matrix(np.eye(3), translate)
    
    def get_Tmat_TranslateInZ(self, direction = 1):     #format
        offset = [0.0, 0.0, np.sign(direction)*self.d_z_normal]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranslateInBodyF(translate = offset)

    def get_Tmat_TranslateInY(self, direction = 1):
        offset = [0.0, np.sign(direction)*self.d_lat, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranslateInBodyF(translate = offset)
    
    def get_Tmat_TranslateInX(self, direction = 1):
        offset = [np.sign(direction)*self.d_lat, 0.0, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranslateInBodyF(translate = offset)
    
    def calculate_unit_vectors(self, num_chambers):
        return [np.array([np.cos(-np.pi / (num_chambers) + 2 * np.pi * i / num_chambers),
                      np.sin(-np.pi / (num_chambers) + 2 * np.pi * i / num_chambers)])
            for i in range(num_chambers)]

    def calculate_direction_vector(self, unit_vectors, vacuum_pressures):
        direction_vector = np.sum([vp * uv for vp, uv in zip(vacuum_pressures, unit_vectors)], axis=0)
        return direction_vector / np.linalg.norm(direction_vector) if np.linalg.norm(direction_vector) > 0 else np.array([0, 0])
    
    def get_lateral_direction_vector(self, P_array, thereshold = True):
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
        unit_vectors = self.calculate_unit_vectors(self.n)
        return self.calculate_direction_vector(unit_vectors, P_array)
        
    def get_Tmat_lateralMove(self, P_array):
        v = self.get_lateral_direction_vector(P_array, True)
        v_step = v * self.d_lat
        # positive x-axis is towards south
        # positive y-axis is towards west
        # positive z-axis is towards down
        # convert the lateral direction vector to the TCP's frame
        return self.get_Tmat_TranslateInBodyF([-v_step[1], -v_step[0], 0.0])
    
        
    def get_Tmats_from_controller(self, P_array, controller_str = "normal"):
        # ["normal","yaw","momentum","momentum_yaw"]
        if controller_str == "normal":
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove(P_array)

        return T_later, T_align

    def get_Tmat_lateralMove_random(self):
        d_lat = self.d_lat
        theta = np.random.rand() * 2*np.pi
        dx_lat = d_lat * np.cos(theta)
        dy_lat = d_lat * np.sin(theta)

        T = self.get_Tmat_TranslateInBodyF([dx_lat, dy_lat, 0.0])
        return T      

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