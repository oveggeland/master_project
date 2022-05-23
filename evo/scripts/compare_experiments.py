from cProfile import label
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from helper import *


def triangulation_errors(exp, id, offsets, stereo=False):
    data = read_pcl_data(exp)

    first = True
    ticks = []

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        ticks.append(BASELINES[bl])

        bl_data = data[bl]
        first_frames = [run[run[:, FRAMEID] == 0] for run in bl_data]
        if stereo:
            first_frames = [run[run[:, TRI] == 1] for run in first_frames]


        points = [np.unique(run[:, POINTID]) for run in first_frames]
        print("Number of triangulated points for", exp, "is", sum([len(element) for element in points]), "for bl", bl)

        for run_idx in range(len(points)):
            run_data = first_frames[run_idx]
            for point_idx in points[run_idx]:
                point_data = run_data[run_data[:, POINTID] == point_idx]
                initial_depth = point_data[0, DIST]

                if first:
                    plt.scatter(BASELINES[bl]+offsets[id], initial_depth, marker='.', s=10, linewidth=5, c=COLORS[id], label=LABELS[id])
                    first = False
                else:
                    plt.scatter(BASELINES[bl]+offsets[id], initial_depth, marker='.', s=10, linewidth=5, c=COLORS[id])
    
    plt.xticks(ticks)
    

def height_errors(exp, id=0, offset=0):
    data = read_traj_data(exp)

    first = True
    ticks = []

    bl_vals = [BASELINES[bl] for bl in sorted(data.keys())]
    seperators = np.diff(bl_vals)/2 + bl_vals[:-1]
    for x in seperators:
        plt.axvline(x)

    for bl in sorted(data.keys()):
        if bl not in BASELINES.keys():
            continue
        ticks.append(BASELINES[bl])

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        height_trajs = [run[:, 1] for run in bl_data]

        max_heights = [max(traj) for traj in height_trajs]

        for height in max_heights:
            if first:
                plt.scatter(BASELINES[bl]+offset, height, marker='.', s=10, linewidth=5, c=COLORS[id], label=LABELS[id])
                first = False
            else:
                plt.scatter(BASELINES[bl]+offset, height, marker='.', s=10, linewidth=5, c=COLORS[id])

    plt.xticks(ticks)

def pos_errors(exp, id=0, offset=0):
    data = read_traj_data(exp)

    first = True
    ticks = []

    bl_vals = [BASELINES[bl] for bl in sorted(data.keys())]
    seperators = np.diff(bl_vals)/2 + bl_vals[:-1]
    for x in seperators:
        plt.axvline(x)

    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        ticks.append(BASELINES[bl])

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        final_pos_values = [run[-1, 1:4] for run in bl_data]

        # Per distance errors, with average and std
        abs_pos_errors = [np.linalg.norm(pos) for pos in final_pos_values]

        # Scatterplot of errors
        if first:
            plt.scatter([BASELINES[bl]+offset for _ in abs_pos_errors], abs_pos_errors, c=COLORS[id], marker=".", s=10, linewidth=5, label=LABELS[id])
            first=False
        else:
            plt.scatter([BASELINES[bl]+offset for _ in abs_pos_errors], abs_pos_errors, c=COLORS[id], marker=".", s=10, linewidth=5)

    
    plt.xticks(ticks)

def compare_triangulation(exps):
    plt.figure("Compare triangulation depths")

    n = len(exps)
    spread = 0.15*n
    offsets = np.linspace(-spread, spread, len(exps))

    for i, exp in enumerate(exps):
        triangulation_errors(exp, id=i, offsets=offsets)

    plt.legend()
    
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Triangulated depth[m]")

    _, y_max = plt.ylim()
    plt.ylim(0, min(y_max, 60))

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "triangulation", "_".join(exps)))


def compare_depth_parameter(exps):
    ticks = []

    heights = {}
    errors = {}
    avgs = {}
    stds = {}
    tri_count = {}
    for i, exp in enumerate(exps):
        traj_data = read_traj_data(exp)
        pcl_data = read_pcl_data(exp)
        init_depth = float(exp[-2:])
        ticks.append(init_depth)
        
        for bl in BASELINES.keys():
            if bl not in pcl_data.keys():
                continue
            bl_pcl_data = np.vstack(pcl_data[bl])
            first_frame = bl_pcl_data[bl_pcl_data[:, FRAMEID] == 0]
            first_frame = first_frame[first_frame[:, TRI] == 1]

            depths = first_frame[:, DIST]
            count = first_frame.shape[0]

            bl_traj_data = traj_data[bl]
            avg_error = np.average([np.linalg.norm(run[0, [1, 2, 3]] - run[-1, [1, 2, 3]]) for run in bl_traj_data])
        

            if bl not in avgs.keys():
                avgs[bl] = [np.average(depths)]
                stds[bl] = [np.std(depths)]
                tri_count[bl] = [count]
                errors[bl] = [avg_error]
            else:
                avgs[bl].append(np.average(depths))
                stds[bl].append(np.std(depths))
                tri_count[bl].append(count)
                errors[bl].append(avg_error)
    

    plt.figure("Compare initial depths")
    for i, bl in enumerate(avgs.keys()):
        #plt.errorbar(ticks, avgs[bl], stds[bl], c=cmap(i))
        plt.plot(ticks, avgs[bl], label=bl, linestyle='dashed')

    plt.legend()
    plt.xticks(ticks)
    for tick in ticks:
        plt.axvline(tick)
    plt.xlabel("Initial depth value[m]")
    plt.ylabel("Average initial depth[m]")

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "initial_depths_"+exps[0][:-2]))


    plt.figure("Compare number of triangulations")
    for bl in avgs.keys():
        plt.plot(ticks, tri_count[bl], label=bl)

    plt.legend()
    plt.xticks(ticks)
    plt.xlabel("Initial depth value[m]")
    plt.ylabel("Number of triangulated features")

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "tri_count_"+exps[0][:-2]))


    plt.figure("Compare total errors")
    for bl in avgs.keys():
        plt.plot(ticks, errors[bl], label=bl, alpha=0.8, linestyle='dashed')
    tot_avg = np.average([errors[bl] for bl in avgs.keys()], axis=0)
    plt.plot(ticks, tot_avg, label="Overall average", alpha=1, linewidth=2.5, c='black')

    plt.legend()
    plt.xticks(ticks)
    plt.xlabel("Initial depth value[m]")
    plt.ylabel("Total position error[m]")

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "pos_error_"+exps[0][:-2]))




def compare_heights(exps, gt):
    plt.figure("Compare height errors")

    n = len(exps)
    spread = 0.15*n
    offsets = np.linspace(-spread, spread, len(exps))

    for i, exp in enumerate(exps):
        height_errors(exp, id=i, offset=offsets[i])

    plt.legend(loc='upper right')

    plt.axhline(gt, linestyle='dashed')
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Height from start[m]")

    y_min, y_max = plt.ylim()
    plt.ylim(max(0, y_min), min(y_max, 30))

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
            plt.scatter(f"run {run_id+1}", pos_error, c=COLORS[i], s=1000, linewidth=5)

        labels.append(mpatches.Patch(color=COLORS[i], label=LABELS[i]))

    plt.legend(handles=labels)
    
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "positions", "_".join(exps)))


def compare_pos_errors(exps):
    plt.figure("Compare position errors")

    n = len(exps)
    spread = 0.15*n
    offsets = np.linspace(-spread, spread, len(exps))

    for i, exp in enumerate(exps):
        pos_errors(exp, offset=offsets[i], id=i)
    
    plt.legend(loc='upper right')

    y_min, y_max = plt.ylim()
    plt.ylim(max(0, y_min), min(y_max, 20))

    plt.xlabel("Baseline[cm]")
    plt.ylabel("Final position error [m]")

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "positions", "_".join(exps)))


COLORS = ['b', 'g', 'r', 'c']
LABELS = ['mono', 'stereo - no cross camera', 'stereo - with cross camera', 'the last option']

if __name__ == "__main__":
    try:
        exps = sys.argv[1:-1]

        try:
            height = float(sys.argv[-1])
        except:
            height = 10
            exps.append(sys.argv[-1])

        print(f"Comparing:")
        for exp in exps:
            print("\t", exp)
    except:
        print("Please provide at least one experiment name and a height reference")
        exit()

    #compare_heights(exps, gt=height)
    #compare_pos_errors(exps)
    #compare_triangulation(exps)
    #compare_each_run(exps, bl='b6')

    compare_depth_parameter(exps)



    plt.show()
