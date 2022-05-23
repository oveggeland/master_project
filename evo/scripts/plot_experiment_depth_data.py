# Script to plot some of the experimental depth data!
from cmath import nan
from distutils.log import error
import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scripts.helper import rotate_vector
import seaborn as sns

from helper import *


def bin_data_by_distance(data):
    x = data[:, DIST]
    bins = np.arange(x.max()) + 0.5
    x_binned = np.digitize(x, bins)
    data[:, DIST] = x_binned
    return data, x_binned.max()+1


def plot_distance_trajectory(data, trajs, plot_path=None):
    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        plt.figure(f"Estimated distances for baseline {bl}")

        bl_traj_data = trajs[bl]
        bl_data = data[bl]
        for i in range(len(bl_data)):
            run = bl_data[i]
            traj = bl_traj_data[i]
            # for each run, plot the trajectory of average depth estimates
            n_frames = int(run[:, FRAMEID].max()-1)
            distances = np.zeros(n_frames)
            for i in range(n_frames):
                frame_data = run[run[:, FRAMEID] == i]

                n_points = frame_data.shape[0]
                frame_dists = np.zeros(n_points)
                for j in range(n_points):
                    vector = frame_data[j, [X, Y, Z]]
                    
                    x, y, z, qx, qy, qz, qw  = traj[i, 1:]
                    world_frame_vector = rotate_vector(vector, [qx, qy, qz, qw])

                    distance = world_frame_vector[0] + x
                    frame_dists[j] = distance

                distances[i] = np.average(frame_dists)

            plt.plot(np.arange(n_frames), distances)
            #plt.yscale('log')
        if plot_path:
            plt.savefig(os.path.join(plot_path, bl, "distance_trajectory.png"))

def plot_depth_trajectory(data, plot_path=None):
    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        plt.figure(f"Estimated depths for baseline {bl}")

        bl_data = data[bl]
        for run in bl_data:
            # for each run, plot the trajectory of average depth estimates
            n_frames = int(run[:, FRAMEID].max()+1)
            depths = np.zeros(n_frames)
            for i in range(n_frames):
                frame_data = run[run[:, FRAMEID] == i]
                depths[i] = np.average(frame_data[:, DIST])

            plt.plot(np.arange(n_frames), depths)
            #plt.yscale('log')

        if plot_path:
            plt.savefig(os.path.join(plot_path, bl, "depth_trajectory.png"))

def initial_distance_per_baseline(data, trajs, gt=10, stereo=False, plot_path=None):
    plt.figure(f"Initial distances from wall")
    plt.axhline(gt, label='gt', linestyle='dashed')

    distance = 0
    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        bl_trajs = trajs[bl]
        bl_data = data[bl]

        first_frames = [run[run[:, FRAMEID] == 0] for run in bl_data]
        if stereo:
            first_frames = [run[run[:, TRI] == 1] for run in first_frames]

        points = [np.unique(run[:, POINTID]) for run in first_frames]

        for run_idx in range(len(points)):
            run_traj = bl_trajs[run_idx]
            run_data = first_frames[run_idx]
            for point_idx in points[run_idx]:
                point_data = run_data[run_data[:, POINTID] == point_idx]
                vector = point_data[0, [X, Y, Z]]
                
                # Find pose and rotate vector
                frame_id = int(point_data[0, FRAMEID])
                x, y, z, qx, qy, qz, qw  = run_traj[frame_id, 1:]
                world_frame_vector = rotate_vector(vector, [qx, qy, qz, qw])

                distance = world_frame_vector[0] + x
                plt.scatter(BASELINES[bl], distance, c='r', alpha=0.5)
    
    plt.scatter(BASELINES[bl], distance, c='r', alpha=0.5, label='Initial points')
    plt.legend()
    
    _, curr_max = plt.ylim()
    plt.ylim(0, min(curr_max, 100))
    plt.ylabel("Distance[m]")
    plt.xlabel("Baseline[cm]")
    if plot_path:
        plt.savefig(os.path.join(plot_path, f"initial_distances.png"))


def initial_depth_per_baseline(data, stereo=False, plot_path=None):
    plt.figure(f"Initial depths")

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        bl_data = data[bl]
        first_frames = [run[run[:, FRAMEID] == 0] for run in bl_data]
        if stereo:
            first_frames = [run[run[:, TRI] == 1] for run in first_frames]

        points = [np.unique(run[:, POINTID]) for run in first_frames]

        for run_idx in range(len(points)):
            run_data = first_frames[run_idx]
            for point_idx in points[run_idx]:
                point_data = run_data[run_data[:, POINTID] == point_idx]
                initial_depth = point_data[0, DIST]

                plt.scatter(BASELINES[bl], initial_depth, c='r')

    plt.ylabel("Triangulated depth[m]")    
    plt.xlabel("Baseline[cm]")        
    if plot_path:
        plt.savefig(os.path.join(plot_path, f"initial_depths.png"))

def initial_distance_distribution(data, traj, gt=10, stereo=False, plot_path=None):
    plt.figure(f"Initial distance distribution")

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        
        bl_trajs = traj[bl]
        bl_data = data[bl]
        first_frames = [run[run[:, FRAMEID] == 0] for run in bl_data]

        if stereo:
            first_frames = [run[run[:, TRI] == 1] for run in first_frames]

        dists = np.zeros((sum([element.shape[0] for element in first_frames])))

        cnt = 0
        for run_idx in range(len(bl_trajs)):
            run_traj = bl_trajs[run_idx]
            run_points = first_frames[run_idx]

            for i in range(run_points.shape[0]):
                point_data = run_points[i, :]
                vector = point_data[[X, Y, Z]]
                
                # Find pose and rotate vector
                frame_id = int(point_data[FRAMEID])
                x, y, z, qx, qy, qz, qw  = run_traj[frame_id, 1:]
                world_frame_vector = rotate_vector(vector, [qx, qy, qz, qw])

                distance = world_frame_vector[0] + x
                dists[cnt] = distance
                cnt+=1

        sns.distplot(dists, label=f"{BASELINES[bl]}cm", hist=False)


    plt.axvline(gt, label='gt', linestyle='dashed')
    x_min, x_max = plt.xlim()
    plt.xlim(max(x_min, gt-10), min(x_max, gt+10))
    plt.ylabel("Probability density")    
    plt.xlabel("Distance triangulated")    
    plt.legend()
    plt.tight_layout()    
    if plot_path:
        plt.savefig(os.path.join(plot_path, f"initial_distance_distribution.png"))

def initial_depth_distribution(data, stereo=False, plot_path=None):
    plt.figure(f"Initial depth distribution")

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        
        bl_data = data[bl]
        first_frames = [run[run[:, FRAMEID] == 0] for run in bl_data]

        if stereo:
            first_frames = [run[run[:, TRI] == 1] for run in first_frames]

        all_data = np.vstack((first_frames))

        dists = all_data[:, DIST]

        sns.distplot(dists, label=f"{BASELINES[bl]}cm", hist=False)

    plt.ylabel("Number")    
    plt.xlabel("Depth triangulated")    
    plt.legend()
    plt.tight_layout()    
    if plot_path:
        plt.savefig(os.path.join(plot_path, f"initial_depth_distribution.png"))




if __name__ == "__main__":
    print("Plot experimental depth data...")
    try:
        exp = sys.argv[1]
        height = float(sys.argv[2])
    except:
        print("please provide an experiment folder and a reference height")
        exit()

    traj_path = os.path.join(EVO_PATH, "data", "trajs", exp)
    data_path = os.path.join(EVO_PATH, "data", "depths", exp)
    plot_path = os.path.join(EVO_PATH, "plots", exp)
    try_create_path(plot_path)


    # Extract data from data folder
    baselines = [name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name)) and name in BASELINES.keys()]
    depth_data = {
        bl:[] for bl in baselines
    }
    traj_data = {
        bl:[] for bl in baselines
    }

    for bl in baselines:
        bl_depth_path = os.path.join(data_path, bl)
        bl_depth_data = []
        bl_traj_path = os.path.join(traj_path, bl)
        bl_traj_data = []

        runs = [name for name in os.listdir(bl_depth_path) if os.path.isfile(os.path.join(bl_depth_path, name))]
        for run in runs:
            run_data = pd.read_csv(os.path.join(bl_depth_path, run), sep=" ").to_numpy()
            bl_depth_data.append(run_data[run_data[:, POINTID] != -1])
        depth_data[bl] = bl_depth_data

        runs = [name for name in os.listdir(bl_traj_path) if os.path.isfile(os.path.join(bl_traj_path, name))]
        for run in runs:
            run_data = pd.read_csv(os.path.join(bl_traj_path, run), sep=" ").to_numpy()
            bl_traj_data.append(run_data)
        traj_data[bl] = bl_traj_data
    
    # Different plot functions to visualize the data
    initial_distance_distribution(depth_data, traj_data, gt=height, stereo=False, plot_path=plot_path)    
    initial_distance_per_baseline(depth_data, traj_data, gt=height, stereo=False, plot_path=plot_path)
    
    initial_depth_distribution(depth_data, stereo=False, plot_path=plot_path)  
    initial_depth_per_baseline(depth_data, stereo=False, plot_path=plot_path)  

    plot_depth_trajectory(depth_data, plot_path=plot_path)
    plot_distance_trajectory(depth_data, traj_data, plot_path)

    plt.show()

