# Script to plot some of the experimental depth data!
from cmath import nan
from distutils.log import error
import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from helper import *


def bin_data_by_distance(data):
    x = data[:, DIST]
    bins = np.arange(x.max()) + 0.5
    x_binned = np.digitize(x, bins)
    data[:, DIST] = x_binned
    return data, x_binned.max()+1


def plot_depth_trajectory(data, plot_path=None):
    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        plt.figure(f"Estimated depths for baseline {bl}")

        bl_data = data[bl]
        for run in bl_data:
            # for each run, plot the trajectory of average depth estimates
            n_frames = int(run[:, FRAMEID].max()+1)
            print(n_frames)
            depths = np.zeros(n_frames)
            for i in range(n_frames):
                frame_data = run[run[:, FRAMEID] == i]
                depths[i] = np.average(frame_data[:, DIST])

            plt.plot(np.arange(n_frames), depths)
            #plt.yscale('log')


def initial_distance_per_baseline(data, frame_count=10, plot_path=None):
    plt.figure(f"Initial depths for the first {frame_count} frames")

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        bl_data = data[bl]
        first_frames = [run[run[:, FRAMEID] <= frame_count] for run in bl_data]
        tri_points = [run[run[:, TRI] == 1] for run in first_frames]

        points = [np.unique(run[:, POINTID]) for run in tri_points]

        for run_idx in range(len(points)):
            run_data = tri_points[run_idx]
            for point_idx in points[run_idx]:

                point_data = run_data[run_data[:, POINTID] == point_idx]
                initial_depth = point_data[0, DIST]

                plt.scatter(bl, initial_depth, c='r')
                
    if plot_path:
        plt.savefig(os.path.join(plot_path, f"initial_depths_{frame_count}_frames.png"))


def average_uncertainty_per_distance(data, plot_path=None, only_tri_points=True, errorbar=False):
    plt.figure("Average std per distance")

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        # Stack data from all runs and remove non triangulated points
        bl_data = data[bl]
        data_stack = np.vstack(([run for run in bl_data]))
        if only_tri_points:
            data_stack = data_stack[data_stack[:, TRI] == 1]
        data_stack = data_stack[(data_stack[:, CAM0ST] == 4) | (data_stack[:, CAM1ST] == 4)]
        data_stack = data_stack[(data_stack[:, DIST] < 30) & (data_stack[:, DCOV] > 0) & (data_stack[:, DCOV] < 100)]

        # Bin on integer distances
        data_binned, n_bins = bin_data_by_distance(data_stack)
        x = np.arange(n_bins)

        # Calculate average and std for each bin
        avgs = np.zeros(n_bins)
        stds = np.zeros(n_bins)
        for val in x:
            bin_data = data_binned[data_binned[:, DIST] == val]
            if bin_data.shape[0]:
                avgs[val] = np.average(np.sqrt(bin_data[:, DCOV]))
                stds[val] = np.std(np.sqrt(bin_data[:, DCOV]))

        valid_indices = (avgs!=0) & (stds!=0)
        x = x[valid_indices]
        avgs = avgs[valid_indices]
        stds = stds[valid_indices]

        # Smooth with polynomial fit
        poly_avgs = fit_poly(x, avgs, 2)

        if errorbar == True:
            plt.errorbar(x, avgs, stds, label=f"{BASELINES[bl]}cm")
        else:
            plt.plot(x, poly_avgs, label=f"{BASELINES[bl]}cm")
            plt.scatter(x, avgs)

    plt.legend()
    plt.xlabel("Distance[m]")
    plt.ylabel("Standard deviation[m]")
    plt.tight_layout()
    if plot_path:
        plt.savefig(os.path.join(plot_path, "distance_uncertainty.png"))


if __name__ == "__main__":
    print("Plot experimental depth data...")
    try:
        exp = sys.argv[1]
    except:
        print("please provide an experiment folder")
        exit()

    data_path = os.path.join(EVO_PATH, "data", "depths", exp)
    plot_path = os.path.join(EVO_PATH, "plots", exp)
    try_create_path(plot_path)


    # Extract data from data folder
    baselines = [name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name)) and name in BASELINES.keys()]
    all_data = {
        bl:[] for bl in baselines
    }
    print(all_data)

    for bl in baselines:
        bl_path = os.path.join(data_path, bl)
        bl_data = []

        runs = [name for name in os.listdir(bl_path) if os.path.isfile(os.path.join(bl_path, name))]
        for run in runs:
            run_data = pd.read_csv(os.path.join(bl_path, run), sep=" ").to_numpy()
            bl_data.append(run_data[run_data[:, POINTID] != -1])

        all_data[bl] = bl_data
    
    # Different plot functions to visualize the data
    #plot_depth_trajectory(all_data, plot_path=None)
    #initial_distance_per_baseline(all_data, plot_path=None)
    initial_distance_per_baseline(all_data, 10, plot_path)
    plt.show()
    #average_uncertainty_per_distance(all_data, plot_path)