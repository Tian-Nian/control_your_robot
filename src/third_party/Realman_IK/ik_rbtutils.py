#! -*-conding=: UTF-8 -*-
#
################################################################################
#                                                                              #
# Author   : Leon                                                              #
# Date     : 2023/11/30                                                        #
# Copyright: Copyright (c) 2018-2024 RealMan Co., Ltd.. All rights reserved.   #
#                                                                              #
################################################################################

from math import pi
import numpy as np
import math

deg2rad = pi/180
rad2deg = 180/pi

def vex(s):
    return np.array([s[2, 1] - s[1, 2], s[0, 2] - s[2, 0], s[1, 0] - s[0, 1]]) / 2

def T2t(T):
    T = np.mat(T)
    t = T[0:3,3]
    return t

def T2r(T):
    T = np.mat(T)
    R = T[0:3,0:3]
    return R

def T_inv(T):
    Ti = np.linalg.inv(T)
    return Ti

def norm(v):
    """
    Norm of vector

    :param v: any vector
    :type v: array_like(n)
    :return: norm of vector
    :rtype: float
    """
    v = np.array(v)
    sum = 0
    for x in v:
        sum += x * x
    return np.sqrt(sum)

def pose_to_matrix_inv(pose):
    T = np.mat(np.eye(4,4))
    x = pose[0]
    y = pose[1]
    z = pose[2]
    rx = pose[3]
    ry = pose[4]
    rz = pose[5]

    s_x = np.sin(rx)
    s_y = np.sin(ry)
    s_z = np.sin(rz)
    c_x = np.cos(rx)
    c_y = np.cos(ry)
    c_z = np.cos(rz)

    T[0,0] = c_y * c_z
    T[0,1] = c_y * s_z
    T[0,2] = -s_y
    T[0,3] = z * s_y - x * c_y * c_z - y * c_y * s_z

    T[1,0] = c_z * s_x * s_y - c_x * s_z
    T[1,1] = c_x * c_z + s_x * s_y * s_z
    T[1,2] = c_y * s_x
    T[1,3] = x * c_x * s_z - y * c_x * c_z - z * c_y * s_x - x * c_z * s_x * s_y - y * s_x * s_y * s_z

    T[2,0] = s_x * s_z + c_x * c_z * s_y
    T[2,1] = c_x * s_y * s_z - c_z * s_x
    T[2,2] = c_x * c_y
    T[2,3] = y * c_z * s_x - z * c_x * c_y - x * s_x * s_z - x * c_x * c_z * s_y - y * c_x * s_y * s_z

    return T

def pose_to_matrix(pose):
    T = np.mat(np.eye(4,4))
    x = pose[0]
    y = pose[1]
    z = pose[2]
    rx = pose[3]
    ry = pose[4]
    rz = pose[5]

    s_x = np.sin(rx)
    s_y = np.sin(ry)
    s_z = np.sin(rz)
    c_x = np.cos(rx)
    c_y = np.cos(ry)
    c_z = np.cos(rz)

    T[0,0] = c_y * c_z
    T[0,1] = c_z * s_x * s_y - c_x * s_z
    T[0,2] = s_x * s_z + c_x * c_z * s_y
    T[0,3] = x

    T[1,0] = c_y * s_z
    T[1,1] = c_x * c_z + s_x * s_y * s_z
    T[1,2] = c_x * s_y * s_z - c_z * s_x
    T[1,3] = y

    T[2,0] = -s_y
    T[2,1] = c_y * s_x
    T[2,2] = c_x * c_y
    T[2,3] = z
    return T

def T2delta_diff(T,Td):
    T  = np.mat(T)
    Td = np.mat(Td)
    Terr = T_inv(T) @ Td
    dX = T2t(Terr)
    dR = vex(T2r(Terr) - np.eye(3)).tolist()
    ret = [dX[0,0],dX[1,0],dX[2,0],dR[0],dR[1],dR[2]]
    return ret

def quat_to_matrix(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    R = [[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]]
    return R

def euler_to_matrix(euler):
    rx = euler[0]
    ry = euler[1]
    rz = euler[2]
    sin_rx = np.sin(rx)
    cos_rx = np.cos(rx)
    sin_ry = np.sin(ry)
    cos_ry = np.cos(ry)
    sin_rz = np.sin(rz)
    cos_rz = np.cos(rz)
    R = [[cos_rz * cos_ry, -sin_rz * cos_rx + cos_rz * sin_ry * sin_rx, sin_rz * sin_rx + cos_rz * sin_ry * cos_rx],
         [sin_rz * cos_ry, cos_rz * cos_rx + sin_rz * sin_ry * sin_rx, -cos_rz * sin_rx + sin_rz * sin_ry * cos_rx],
         [-sin_ry, cos_ry * sin_rx, cos_ry * cos_rx]]
    return R

def iszerovec(v):
    """
    Test if vector has zero length

    :param v: vector to test
    :type v: ndarray(n)
    :param tol: tolerance in units of eps
    :type tol: float
    :return: whether vector has zero length
    :rtype: bool
    """
    return np.linalg.norm(v) < 1e-16

def angle_axis_diff(T, Td):
    d_ = T2t(Td) - T2t(T)
    d = [d_[0,0],d_[1,0],d_[2,0]]
    R = T2r(Td) @ T2r(T).T
    li = np.r_[R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]

    if iszerovec(li):
        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    return np.r_[d, a]
