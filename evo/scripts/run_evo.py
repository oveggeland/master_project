# Script to read all rovio data at once!
import sys
import os
import glob
import json
import struct
import rosbag
from helper import *


def run_evo(rel_dir):
    print("---------", rel_dir, "---------")
    # Create folders they don't exist
    plot_path = os.path.join(EVO_PATH, "plots", rel_dir)
    try_create_path(plot_path)

    # Run evo with tum files in this folder!
    folder = os.path.join(EVO_PATH, "data", "trajs", rel_dir)
    os.system(f"evo_traj tum {folder}/*.txt --save_plot {plot_path}/evo --plot_mode xy")

    # Recursively continue into subfolders
    sub_folders = [name for name in os.listdir(rel_dir) if os.path.isdir(os.path.join(rel_dir, name))]
    for folder in sub_folders:
        if folder != "rovio":
            new_rel_dir = os.path.join(rel_dir, folder)
            run_evo(new_rel_dir)


if __name__ == "__main__":
    print("Running evo")
    try:
        exp = sys.argv[1]
    except:
        print("Please provide an experiment name")
        exit()

    os.chdir(os.path.join(EVO_PATH, "data", "trajs"))
    try_create_path(os.path.join(EVO_PATH, "plots", "evo"))
    run_evo(exp)
