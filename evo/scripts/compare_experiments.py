import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from helper import *


def triangulation_errors(exp, color, alpha, frame_count=1):
    data = read_pcl_data(exp)

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

                plt.scatter(BASELINES[bl], initial_depth, c=color, alpha=alpha)
    

def height_errors(exp, color, alpha):
    data = read_traj_data(exp)

    for bl in sorted(data.keys()):
        if bl not in BASELINES.keys():
            continue

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        height_trajs = [run[:, 1] for run in bl_data]

        max_height = [max(traj) for traj in height_trajs]

        plt.scatter([BASELINES[bl] for _ in height_trajs], max_height, marker='_', s=1000, linewidth=5, c=color, alpha=alpha)


def pos_errors(exp, color, alpha):
    data = read_traj_data(exp)

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        final_pos_values = [run[-1, 1:4] for run in bl_data]

        # Per distance errors, with average and std
        abs_pos_errors = [np.linalg.norm(pos) for pos in final_pos_values]

        # Scatterplot of errors
        plt.scatter([BASELINES[bl] for i in abs_pos_errors], abs_pos_errors, c=color, alpha=alpha, marker="_", s=1000, linewidth=5)

def compare_triangulation(exp1, exp2, labels=None):
    if not labels:
        labels = [exp1, exp2]

    fig = plt.figure("Compare triangulation depths")

    triangulation_errors(exp1, color='b', alpha=0.3)
    triangulation_errors(exp2, color='r', alpha=0.3)

    red_patch = mpatches.Patch(color='red', label=labels[0])
    blue_patch = mpatches.Patch(color='blue', label=labels[1])
    plt.legend(handles=[red_patch, blue_patch])
    
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Triangulated depth[m]")
    plt.ylim(0, 50)
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "triangulation", f"{exp1}_vs_{exp2}"))
    plt.show()


def compare_heights(exp1, exp2, gt=10):
    fig = plt.figure("Compare height errors")
    height_errors(exp1, color='b', alpha=1)
    height_errors(exp2, color='r', alpha=1)
    plt.axhline(gt, linestyle='dashed')

    plt.xlabel("Baseline[cm]")
    plt.ylabel("Height from start[m]")

    red_patch = mpatches.Patch(color='red', label="With cluster filter")
    blue_patch = mpatches.Patch(color='blue', label="Regular")
    plt.legend(handles=[red_patch, blue_patch])
    
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "heights", f"{exp1}_vs_{exp2}"))

def compare_errors(exp1, exp2):
    fig = plt.figure("Compare position errors")
    pos_errors(exp1, color='b', alpha=1)
    pos_errors(exp2, color='r', alpha=1)

    plt.xlabel("Baseline[cm]")
    plt.ylabel("Final position error [m]")

    red_patch = mpatches.Patch(color='red', label="With cluster filter")
    blue_patch = mpatches.Patch(color='blue', label="Regular")
    plt.legend(handles=[red_patch, blue_patch])
    
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "positions", f"{exp1}_vs_{exp2}"))

if __name__ == "__main__":
    print("Compare two experiments")

    try:
        exp1, exp2 = sys.argv[1:3]
        print(f"Comparing {exp1} and {exp2}")
    except:
        print("Please provide two experiment names")
        exit()

    compare_heights(exp1, exp2)
    compare_errors(exp1, exp2)
    if len(sys.argv) >= 5:
        compare_triangulation(exp1, exp2, [sys.argv[3], sys.argv[4]])

    plt.show()
