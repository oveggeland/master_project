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
        print("Number of triangulated points are", sum([len(element) for element in points]), "for bl", bl)

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
        print(bl_data)
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

def compare_triangulation(exps):
    plt.figure("Compare triangulation depths")

    labels=[]
    for i, exp in enumerate(exps):
        triangulation_errors(exp, color=COLORS[i], alpha=0.3)
        labels.append(mpatches.Patch(color=COLORS[i], label=LABELS[i]))
    plt.legend(handles=labels)
    
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Triangulated depth[m]")

    _, y_max = plt.ylim()
    plt.ylim(0, min(y_max, 60))

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "triangulation", "_".join(exps)))


def compare_heights(exps, gt):
    plt.figure("Compare height errors")

    labels = []
    for i, exp in enumerate(exps):
        height_errors(exp, color=COLORS[i], alpha=1)
        labels.append(mpatches.Patch(color=COLORS[i], label=LABELS[i]))

    plt.legend(handles=labels)

    plt.axhline(gt, linestyle='dashed')
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Height from start[m]")

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "heights", "_".join(exps)))


def compare_each_run(exps, bl='b6'):
    plt.figure("Compare position errors for each run")

    labels = []
    for i, exp in enumerate(exps):
        data = read_traj_data(exp)[bl]

        for run_id in range(len(data)):
            final_pos_val = data[run_id][-1, 1:4]
            pos_error = np.linalg.norm(final_pos_val)
            plt.scatter(f"run {run_id+1}", pos_error, c=COLORS[i], marker="_", s=1000, linewidth=5)

        labels.append(mpatches.Patch(color=COLORS[i], label=LABELS[i]))

    plt.legend(handles=labels)
    
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "positions", "_".join(exps)))


def compare_pos_errors(exps):
    plt.figure("Compare position errors")

    labels = []
    for i, exp in enumerate(exps):
        pos_errors(exp, color=COLORS[i], alpha=1)
        labels.append(mpatches.Patch(color=COLORS[i], label=LABELS[i]))
    plt.legend(handles=labels)

    plt.xlabel("Baseline[cm]")
    plt.ylabel("Final position error [m]")
    
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "positions", "_".join(exps)))


COLORS = ['b', 'g', 'r', 'c']
LABELS = ['something', 'another thing', 'a new idea', 'the last option']

if __name__ == "__main__":
    print("Compare two experiments")

    try:
        exps = sys.argv[1:]
        print(f"Comparing")
        for exp in exps:
            print(exp)
    except:
        print("Please provide an experiment names")
        exit()

    compare_heights(exps, gt=10)
    compare_pos_errors(exps)
    compare_triangulation(exps)
    compare_each_run(exps, bl='b6')



    plt.show()
