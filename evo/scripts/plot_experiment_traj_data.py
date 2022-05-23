# Script to plot some of the experimental depth data!
from cmath import nan
from distutils.log import error
import sys
import os
from tkinter import BASELINE
from matplotlib import markers
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from helper import *


def calculate_traj_length(data):
    pos_data = data[:, 1:4]
    diffs = np.diff(pos_data, axis=0)
    norms = np.linalg.norm(diffs, axis=1)
    return norms.sum()

def get_final_yaw_value(run, deg=True):
    quat = run[-1, 4:]
    roll, pitch, yaw = quat_to_euler(quat, deg=deg)
    return pitch


def yaw_errors_per_distance_traveled(data, plot_path, abs_errors=True):
    plt.figure("Yaw errors")
    ticks = []

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        ticks.append(BASELINES[bl])

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        traj_lengths = [calculate_traj_length(run) for run in bl_data]
        if abs_errors:
            final_yaw_values = [abs(get_final_yaw_value(run)) for run in bl_data]
        else:
            final_yaw_values = [get_final_yaw_value(run) for run in bl_data]

        # Per distance errors, with average and std
        yaw_per_dists = [final_yaw_values[i]/traj_lengths[i] for i in range(len(traj_lengths))]
        avg_yaw_per_dist = np.average(yaw_per_dists)
        std_yaw_per_dist = np.std(yaw_per_dists)

        # Scatter and errorbar
        plt.scatter([BASELINES[bl] for i in yaw_per_dists], yaw_per_dists)
        plt.errorbar(BASELINES[bl], avg_yaw_per_dist, std_yaw_per_dist, barsabove=True, c='r')
        plt.scatter(BASELINES[bl], avg_yaw_per_dist, marker='_', c='r')

    plt.xticks(ticks)
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Yaw error per distance [deg/m]")
    plt.tight_layout()
    if plot_path:
        plt.savefig(os.path.join(plot_path, "yaw_errors_per_distance_traveled.png"))

def pos_errors_per_distance_traveled(data, plot_path):
    plt.figure("Position errors per distance traveled")
    ticks = []

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        ticks.append(BASELINES[bl])

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        traj_lengths = [calculate_traj_length(run) for run in bl_data]
        final_pos_values = [run[-1, 1:4] for run in bl_data]

        # Per distance errors, with average and std
        abs_pos_per_dists = [np.linalg.norm(final_pos_values[i])/traj_lengths[i] for i in range(len(final_pos_values))]
        avg_pos_per_dist = np.average(abs_pos_per_dists)
        std_pos_per_dist = np.std(abs_pos_per_dists)

        # Scatter and errorbar
        plt.scatter([BASELINES[bl] for i in abs_pos_per_dists], [el*100 for el in abs_pos_per_dists])
        plt.errorbar(BASELINES[bl], avg_pos_per_dist*100, std_pos_per_dist*100, barsabove=True, c='r')
        plt.scatter(BASELINES[bl], avg_pos_per_dist*100, marker='_', c='r')

    plt.xticks(ticks)
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Final position per distance traveled [%]")
    plt.tight_layout()
    if plot_path:
        plt.savefig(os.path.join(plot_path, "pos_errors_per_distance_traveled.png"))

def pos_errors(data, plot_path, avg=False):
    plt.figure("Position errors")
    ticks = []

    bls = []
    avgs = []
    stds = []
    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        ticks.append(BASELINES[bl])
        bls.append(BASELINES[bl])
        
        # Get traj lengths and final yaw values
        bl_data = data[bl]
        traj_lengths = [calculate_traj_length(run) for run in bl_data]
        final_pos_values = [run[-1, 1:4] for run in bl_data]

        # Per distance errors, with average and std
        abs_pos_errors = [np.linalg.norm(pos) for pos in final_pos_values]
        avg_pos_error = np.average(abs_pos_errors)
        std_pos_error = np.std(abs_pos_errors)

        # Scatter and errorbar
        if not avg:
            plt.scatter([BASELINES[bl] for i in abs_pos_errors], abs_pos_errors)
            plt.errorbar(BASELINES[bl], avg_pos_error, std_pos_error, barsabove=True, c='r')
            plt.scatter(BASELINES[bl], avg_pos_error, marker='_', c='r')
            
        else:
            avgs.append(avg_pos_error)
            stds.append(std_pos_error)

    if avg:
        plt.figure("Average position errors")
        plt.plot(bls, avgs, 'or', label="Average error")
        plt.plot(bls, avgs, c='black')
        plt.fill_between(bls, np.subtract(avgs, stds), np.add(avgs, stds),
                 color='gray', alpha=0.2, label="Standard deviation")
        plt.legend()

    plt.xticks(ticks)
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Final position error [m]")
    plt.tight_layout()
    if plot_path:
        if avg:
            plt.savefig(os.path.join(plot_path, "avg_pos_errors.png"))
        else:
            plt.savefig(os.path.join(plot_path, "pos_errors.png"))

def height_errors(data, plot_path, gt=10):
    plt.figure("Height errors")
    ticks = []

    for bl in sorted(data.keys()):
        if bl not in BASELINES.keys():
            continue
        ticks.append(BASELINES[bl])

        plt.figure("Height errors")

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        y_trajs = [run[:, 1] for run in bl_data]


        y_length = [max(traj) for traj in y_trajs]

        avg = np.average(y_length)
        std = np.std(y_length)

        plt.errorbar(BASELINES[bl], avg, std, c='r', marker='_')
        plt.plot([BASELINES[bl] for _ in y_trajs], y_length, marker='o')

        plt.xlabel("baseline[cm]")
        plt.ylabel("Translation in y direction")

    plt.plot([12.5, 25], [float(gt), float(gt)], linestyle='dashed')

    plt.xticks(ticks)
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Y translation[m]")
    plt.tight_layout()
    if plot_path:
        plt.savefig(os.path.join(plot_path, "height_errors.png"))


if __name__ == "__main__":
    print("Plot experimental traj data...")
    try:
        exp = sys.argv[1]
        height = float(sys.argv[2])
    except:
        print("please provide an experiment folder and a reference height")
        exit()

    data_path = os.path.join(EVO_PATH, "data", "trajs", exp)
    plot_path = os.path.join(EVO_PATH, "plots", exp)
    try_create_path(plot_path)


    # Extract data from data folder
    baselines = [name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name)) and name in BASELINES.keys()]
    all_data = {
        bl:[] for bl in baselines
    }

    for bl in baselines:
        bl_path = os.path.join(data_path, bl)
        bl_data = []

        runs = [name for name in os.listdir(bl_path) if os.path.isfile(os.path.join(bl_path, name))]
        for run in runs:
            run_data = pd.read_csv(os.path.join(bl_path, run), sep=" ").to_numpy()
            bl_data.append(run_data[run_data[:, POINTID] != -1])

        all_data[bl] = bl_data
    
    # Different plot functions to visualize the data
    #yaw_errors_per_distance_traveled(all_data, plot_path, abs_errors=True)
    #pos_errors_per_distance_traveled(all_data, plot_path)
    pos_errors(all_data, plot_path, avg=True)
    pos_errors(all_data, plot_path, avg=False)
    #height_errors(all_data, plot_path, gt=height)

    plt.show()