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
from scipy.spatial.transform import Slerp
from os import listdir
from os.path import isfile, join

def main():
    args = sys.argv[1:]
    if(len(args) != 2):
        print("Usage: python3 interpolate_poses.py /path_to_trajectory_file /path_to_rgb_folder")
        return

    # 1. read rgb image folder, store as list of timestamps
    # 2. read trajectory file, store as an ordereddict of poses (4x4)
    # 3. for each rgb image timestamp, interpolate its pose, convert to tum format, output to new file
    
    trajectory = {}
    trajectory_file = args[0]
    with open(trajectory_file) as file:
        for line in file:
            values = line.rstrip().split()
            timestamp = float(values[0])
            params = [float(values[4]), float(values[5]), float(values[6]), float(values[7]), float(values[1]), float(values[2]), float(values[3])]
            T_c1_w = paramsToMatrix(params)
            trajectory[timestamp] = T_c1_w

    rgd_folder = args[1]
    outfile = trajectory_file.rstrip('.txt') + '-interpolated.txt'
    file_out = open(outfile, 'w')

    kf_trajectory = {}
    onlyfiles = [f for f in listdir(rgd_folder) if isfile(join(rgd_folder, f))]
    for file in onlyfiles:
        file = file.replace(".png", '')
        time = float(file)
        kf_trajectory[time] = None
    
    sorted_traj = sorted(trajectory)
    sorted_kf_traj = sorted(kf_trajectory)

    for time in sorted_kf_traj:
        # get timestamp just below time and just above it in the trajectory
        if time < sorted_traj[0]:
            continue
        
        if time > sorted_traj[len(sorted_traj) -1]:
            break

        under = sorted_traj[0]
        over = sorted_traj[0]
        for t in sorted_traj:
            if t > time:
                over = t
                break
            else:
                under = t

        T_c1_w_over = trajectory[over]
        T_c1_w_under = trajectory[under]

        w2 = 1.0 * (time - under) / (over - under)

        R1 = R.from_matrix(T_c1_w_under[0:3,0:3])
        R2 = R.from_matrix(T_c1_w_over[0:3,0:3])

        key_rots = R.random(2, random_state=2342345)
        key_rots[0] = R1
        key_rots[1] = R2
        key_times = [under, over]
        slerp = Slerp(key_times, key_rots)
        times = [time]
        interp_rots = slerp(times)

        q = interp_rots[0].as_quat()

        tr1 = np.zeros((3,1))
        tr1[0] = T_c1_w_under[0, 3]
        tr1[1] = T_c1_w_under[1, 3] 
        tr1[2] = T_c1_w_under[2, 3]

        tr2 = np.zeros((3,1))
        tr2[0] = T_c1_w_over[0, 3]
        tr2[1] = T_c1_w_over[1, 3] 
        tr2[2] = T_c1_w_over[2, 3]

        tr = (1 - w2) * tr1 + w2 * tr2
        
        params_out = f'{time:.9f}' + " " + f'{tr[0,0]:.20f}' + " " + f'{tr[1,0]:.20f}' + " " + f'{tr[2,0]:.20f}' + " " + f'{q[0]:.20f}' + " " + f'{q[1]:.20f}' + " " + f'{q[2]:.20f}' + " " + f'{q[3]:.20f}' + "\n"
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