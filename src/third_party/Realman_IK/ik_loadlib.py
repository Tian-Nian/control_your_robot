#! /usr/bin/env python3
#! -*-conding=: UTF-8 -*-
#
################################################################################
#                                                                              #
# Author   : Leon                                                              #
# Date     : 2023/11/30                                                        #
# Copyright: Copyright (c) 2018-2024 RealMan Co., Ltd.. All rights reserved.   #
#                                                                              #
################################################################################

import os
import ctypes
from ctypes import cdll
from enum import IntEnum

lib_paths = {
    "windows": "./lib/librman_algorithm.dll",
    "linux": "./lib/librman_algorithm.so",
    "macOS": "./lib/librman_algorithm.dylib"
}

lib_name = ''
if os.name == 'nt':
    lib_name = lib_paths['windows']
    if not os.path.exists(lib_name):
        raise Exception(f"Error: {lib_name} does not exist.".format(lib_name))
else:
    if os.name == 'posix':
        if 'DARWIN' in os.uname():
            lib_name = lib_paths['macOS']
            if not os.path.exists(lib_name):
                raise Exception(f"Error: {lib_name} does not exist.".format(lib_name))
        else:
            lib_name = lib_paths['linux']
            if not os.path.exists(lib_name):
                    raise Exception(f"Error: {lib_name} does not exist.".format(lib_name))
    else:
        raise Exception(f"Unknown operating system detected.")

lib = []
if os.name == 'nt':
    lib = ctypes.CDLL(lib_name,winmode=0)
else:
    lib = cdll.LoadLibrary(lib_name)

class RobotType(IntEnum):
    RM65 = 0
    RM75 = 1
    RML63II = 3
    ECO65 = 5
    GEN72 = 7

class SensorType(IntEnum):
    B = 0
    ZF = 1
    SF = 2

func = lib.ikine
func.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float * 8), ctypes.POINTER(ctypes.c_float * 8), ctypes.POINTER(ctypes.c_float * 16), ctypes.POINTER(ctypes.c_float * 7), ctypes.POINTER(ctypes.c_float * 7)]
func.restype = ctypes.c_int


def lib_ikine(rbt_type, sr_type, q_ref:list, Ttarget:list, q_max, q_min, is_joint_space = False):
    if rbt_type == RobotType.RM75 or rbt_type == RobotType.GEN72:
        q_in = (ctypes.c_float * 7)(q_ref[0], q_ref[1], q_ref[2], q_ref[3], q_ref[4], q_ref[5], q_ref[6])
        q_max_c = (ctypes.c_float * 8)(q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], q_max[6], 0)
        q_min_c = (ctypes.c_float * 8)(q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], q_min[6], 0)
    else:
        q_in = (ctypes.c_float * 7)(q_ref[0], q_ref[1], q_ref[2], q_ref[3], q_ref[4], q_ref[5], 0)
        q_max_c = (ctypes.c_float * 8)(q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], 0, 0)
        q_min_c = (ctypes.c_float * 8)(q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], 0, 0)
    T = (ctypes.c_float * 16)(Ttarget[0][0],Ttarget[0][1],Ttarget[0][2],Ttarget[0][3],
                              Ttarget[1][0],Ttarget[1][1],Ttarget[1][2],Ttarget[1][3],
                              Ttarget[2][0],Ttarget[2][1],Ttarget[2][2],Ttarget[2][3],
                              Ttarget[3][0],Ttarget[3][1],Ttarget[3][2],Ttarget[3][3])
    q_out = (ctypes.c_float * 7)(0,)
    ret = func(rbt_type, sr_type, is_joint_space, q_max_c, q_min_c, T, q_in, q_out)
    return list(q_out),ret

if __name__ == "__main__":
    q_in = [1.04719755119660, 0.757472895365539, -1.75056523975031, 0, -0.523598775598299, 0]
    T = [[-0.0270394064923877, 0.866025403784439, 0.499268335163106, 0.0719079041256672],
         [-0.0468336258513231, -0.500000000000000, 0.864758123112826, 0.124548143411447],
         [0.998536670326212, 5.55111512312578e-17, 0.0540788129847752, 0.548971876651408],
         [0, 0, 0, 1]]

    q_out,ret = lib_ikine(RobotType.RM65, SensorType.B, q_in, T, 0)
    print(ret) 
    print([x*180/3.14 for x in q_in])
    print([x*180/3.14 for x in q_out])
