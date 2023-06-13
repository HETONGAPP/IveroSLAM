#!/usr/bin/env python2
"""
Save keyframes to folder given a keyframe trajectory
"""

import rosbag
import rospy
import sys
from collections import OrderedDict
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    args = sys.argv[1:]
    if(len(args) != 8):
        print("Usage: python3 transform_trajectory.py /path_to_trajectory_file (trajectory file is expressed as T_world_c1, in the TUM format) qx qy qz qw x y z (T_c2_c1)")
        return

    extrinsic_params = [float(args[1]), float(args[2]), float(args[3]), float(args[4]), float(args[5]), float(args[6]), float(args[7])]
    T_c2_c1 = paramsToMatrix(extrinsic_params)
    
    trajectory_file = args[0]
    outfile = trajectory_file.rstrip('.txt') + '-transformed.txt'
    file_out = open(outfile, 'w')
    with open(trajectory_file) as file:
        for line in file:
            values = line.rstrip().split()
            timestamp = values[0]
            params = [float(values[4]), float(values[5]), float(values[6]), float(values[7]), float(values[1]), float(values[2]), float(values[3])]
            T_w_c1 = paramsToMatrix(params)
            T_c2_w = np.matmul(T_c2_c1, np.linalg.inv(T_w_c1))
            T_w_c2 = np.linalg.inv(T_c2_w)
            
            r = R.from_matrix(T_w_c2[0:3,0:3])
            t = np.zeros((3,1))
            t[0] = T_w_c2[0, 3]
            t[1] = T_w_c2[1, 3] 
            t[2] = T_w_c2[2, 3]
            q = r.as_quat()
            params_out = timestamp + " " + f'{t[0,0]:.20f}' + " " + f'{t[1,0]:.20f}' + " " + f'{t[2,0]:.20f}' + " " + f'{q[0]:.20f}' + " " + f'{q[1]:.20f}' + " " + f'{q[2]:.20f}' + " " + f'{q[3]:.20f}' + "\n"
            file_out.write(params_out)

def paramsToMatrix(params):
    r = R.from_quat([params[0], params[1], params[2], params[3]])
    t = np.array([params[4], params[5], params[6]])
    T_mat = np.zeros((4,4))
    T_mat[0:3,0:3] = r.as_matrix()
    T_mat[0, 3] = t[0]
    T_mat[1, 3] = t[1]
    T_mat[2, 3] = t[2]
    T_mat[3, 3] = 1
    return T_mat

if __name__ == '__main__':
    main()