# Script to plot some of the experimental depth data!
from cmath import nan
from distutils.log import error
from email import header
from mimetypes import init
import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scripts.helper import *

def read_traj(path):
    df = pd.read_csv(path, sep=" ", header=None)
    df.columns = ["time", "x", "y", "z", "qx", "qy", "qz", "qw"]

    return df

def T_imu_w(traj, frame_id):
    transform = traj.iloc[int(frame_id)]

    translation = transform[1:4]
    quat = transform[4:]
    rot_mat = R.from_quat(quat).as_matrix()

    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, 3] = translation
    return T

def convert_to_world_frame(traj, point):
    T_init = T_imu_w(traj, point['frame_id'])
    imu_init_point = np.hstack((point.loc[['x', 'y', 'z']].to_numpy(), np.ones(1)))

    return T_init @ imu_init_point

def plot_point_uncertaintes(all_data):
    for bl in all_data.keys():
        if bl not in BASELINES.keys():
            continue

        tri_errors = []
        notri_errors = []

        for i, run in enumerate(all_data[bl]["pcl"]):
            traj = read_traj(all_data[bl]["traj"][i])
            data = pd.read_csv(run, sep=" ", usecols=['frame_id', 'id', 'x', 'y', 'z', 'd', 'c_00', 'c_11', 'c_22', 'c_d', 'tri', 'init'])
            if data.empty:
                continue

            # Sort on points
            n_points = int(data['id'].max())
            for i in range(n_points+1):
                point_data = data[data['id'] == i].sort_values('frame_id')
                init_point, last_point = point_data.iloc[0], point_data.iloc[-1]

                w_init_point = convert_to_world_frame(traj, init_point)
                w_last_point = convert_to_world_frame(traj, last_point)
        
                diff = (w_init_point - w_last_point)[:3]
                error = np.linalg.norm(diff)
                if error > 10:
                    continue

                if point_data.iloc[0]["tri"]:
                    tri_errors.append(error)
                else:
                    notri_errors.append(error)

            frame_data = data[data['frame_id'] == 100]
            for index, row in frame_data.iterrows():
                w_p = convert_to_world_frame(traj, row)
                plt.scatter(index, w_p[1])

            plt.show()


        plt.figure("Per baseline")
        plt.errorbar(BASELINES[bl], np.average(tri_errors), np.std(tri_errors), marker="_", c='r')
        plt.errorbar(BASELINES[bl], np.average(notri_errors), np.std(notri_errors), marker="_", c='b')

    plt.legend(["stereo init", "no stereo init"])
    plt.show()
            
            

def plot_average_point_distance(all_data):
    for bl in all_data.keys():
        if bl not in BASELINES.keys():
            continue
    pass

if __name__ == "__main__":
    print("Plot experimental depth data...")
    try:
        exp = sys.argv[1]
    except:
        print("please provide an experiemnt folder")
        exit()

    data_path = os.path.join(EVO_PATH, "data", "pcl", exp)
    traj_path = os.path.join(EVO_PATH, "data", "trajs", exp)
    plot_path = os.path.join(EVO_PATH, "plots", exp)
    try_create_path(plot_path)


    # Extract data from data folder
    baselines = [name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name)) and name in BASELINES.keys()]
    all_data = {
        bl:{} for bl in baselines
    }

    for bl in baselines:
        bl_path = os.path.join(data_path, bl)
        all_data[bl]["pcl"] = [os.path.join(bl_path, name) for name in os.listdir(bl_path) if os.path.isfile(os.path.join(bl_path, name))]
        all_data[bl]["traj"] = [os.path.join(traj_path, bl, name) for name in os.listdir(bl_path) if os.path.isfile(os.path.join(traj_path, bl, name))]
    
    
    # Different plot functions to visualize the data
    plot_point_uncertaintes(all_data)

    if len(sys.argv) > 2 and sys.argv[2] == "--dont_show":
        pass
    else:
        plt.show()