# Script to plot some of the experimental depth data!
from distutils.log import error
import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from helper import *

def fit_poly(x, y, dim):
    model = np.polyfit(x, y, dim)
    p = np.poly1d(model)
    return p(x)

def bin_data_by_distance(data):
    x = data[:, DIST]
    bins = np.arange(x.max()) + 0.5
    x_binned = np.digitize(x, bins)
    data[:, DIST] = x_binned
    return data, x_binned.max()+1

def average_uncertainty_per_distance(data, only_tri_points=True, errorbar=False):
    plt.figure("Average std per distance")

    for bl in data.keys():
        # Stack data from all runs and remove non triangulated points
        bl_data = data[bl]
        data_stack = np.vstack(([run for run in bl_data]))
        if only_tri_points:
            data_stack = data_stack[data_stack[:, TRI] == 1]
        data_stack = data_stack[(data_stack[:, CAM0ST] == 4) | (data_stack[:, CAM1ST] == 4)]
        data_stack = data_stack[(data_stack[:, DIST] < 100) & (data_stack[:, DCOV] < 100)]

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

        # Smooth with polynomial fit
        avgs = fit_poly(x, avgs, 5)

        if errorbar == True:
            plt.errorbar(x, avgs, stds, label=bl)
        else:
            plt.plot(x, avgs, label=bl)

    plt.legend()
    plt.xlabel("Distance")
    plt.ylabel("Estimated distance deviation")
    plt.tight_layout()
    


if __name__ == "__main__":
    print("Plot experimental depth data...")
    try:
        exp = sys.argv[1]
    except:
        print("please provide an experiemnt folder")
        exit()

    data_path = os.path.join(EVO_PATH, "data", "depths", exp)
    plt_path = os.path.join(EVO_PATH, "plots", exp)


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
    average_uncertainty_per_distance(all_data)

    plt.show()