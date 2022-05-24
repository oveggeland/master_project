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

def pos_errors(exp, id=0, offset=0, average=False):
    data = read_traj_data(exp)

    first = True
    ticks = []

    bl_vals = [BASELINES[bl] for bl in sorted(data.keys())]
    seperators = np.diff(bl_vals)/2 + bl_vals[:-1]
    for x in seperators:
        plt.axvline(x)
    
    if average:
        plt.axvline(seperators[-1]+2.5)

    errors = 0
    cnt = 0
    for bl in BASELINES.keys():
        if bl not in data.keys():
            continue
        ticks.append(BASELINES[bl])

        # Get traj lengths and final yaw values
        bl_data = data[bl]
        final_pos_values = [run[-1, 1:4] for run in bl_data]

        # Per distance errors, with average and std
        abs_pos_errors = [np.linalg.norm(pos) for pos in final_pos_values]
        errors += sum(abs_pos_errors)
        cnt += len(abs_pos_errors)

        # Scatterplot of errors
        if first:
            plt.scatter([BASELINES[bl]+offset for _ in abs_pos_errors], abs_pos_errors, c=COLORS[id], marker=".", s=10, linewidth=5, label=LABELS[id])
            first=False
        else:
            plt.scatter([BASELINES[bl]+offset for _ in abs_pos_errors], abs_pos_errors, c=COLORS[id], marker=".", s=10, linewidth=5)

    plt.xticks(ticks)
    if average:
        plt.scatter(max(plt.xticks()[0])+2.5, errors/cnt, marker="_", c=COLORS[id], s=1000, linewidth=5)

def compare_triangulation(exps, offset=True):
    plt.figure("Compare triangulation depths")

    n = len(exps)
    if offset:
        spread = 0.15*n
        offsets = np.linspace(-spread, spread, len(exps))
    else:
        offsets = [0]*n

    for i, exp in enumerate(exps):
        triangulation_errors(exp, id=i, offsets=offsets)

    plt.legend()
    
    plt.xlabel("Baseline[cm]")
    plt.ylabel("Triangulated depth[m]")

    _, y_max = plt.ylim()
    plt.ylim(0, min(y_max, 60))

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "triangulation", "_".join(exps)))


def compare_depth_parameter(exps, overall=True, markers=['main10', 'main20', 'main25']):
    ticks = {}

    errors = {}
    avgs = {}
    stds = {}
    tri_count = {}

    labels = ['Initial distance = 10m', 'Initial distance = 20m', 'Initial distance = 25m']
    for mark in markers:
        mark_exps = [exp for exp in exps if mark in exp]

        errors[mark] = {}
        avgs[mark] = {}
        stds[mark] = {}
        tri_count[mark] = {}
        ticks[mark] = []

       
        for i, exp in enumerate(mark_exps):
            traj_data = read_traj_data(exp)
            pcl_data = read_pcl_data(exp)
            init_depth = float(exp[-2:])
            ticks[mark].append(init_depth)
            
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
            

                if bl not in avgs[mark].keys():
                    avgs[mark][bl] = [np.average(depths)]
                    stds[mark][bl] = [np.std(depths)]
                    tri_count[mark][bl] = [count]
                    errors[mark][bl] = [avg_error]
                else:
                    avgs[mark][bl].append(np.average(depths))
                    stds[mark][bl].append(np.std(depths))
                    tri_count[mark][bl].append(count)
                    errors[mark][bl].append(avg_error)
    
    all_ticks = np.hstack([ticks[mark] for mark in markers])
    all_ticks = np.unique(all_ticks)

    plt.figure("Compare initial depths")

    for i, mark in enumerate(markers):
        if not overall:
            for i, bl in enumerate(avgs[mark].keys()):
                #plt.errorbar(ticks[mark], avgs[mark][bl], stds[mark][bl])
                plt.plot(ticks[mark], avgs[mark][bl], label=bl)
        else:
            tot_avg = np.average([avgs[mark][bl] for bl in avgs[mark].keys()], axis=0)
            plt.plot(ticks[mark], tot_avg, label=labels[i], alpha=1, linewidth=2.5)

    plt.legend()
    plt.xticks(all_ticks)
    plt.xlabel("Initial depth value[m]")
    plt.ylabel("Average initial depth[m]")

    plt.tight_layout()
    if overall:
        plt.savefig(os.path.join(EVO_PATH, "compare", "initdepth", "initial_depths_overall"))
    else:
        plt.savefig(os.path.join(EVO_PATH, "compare", "initdepth", "initial_depths_"+exps[0][:-2]))


    plt.figure("Compare number of triangulations")
    for i, mark in enumerate(markers):
        if not overall:
            for i, bl in enumerate(tri_count[mark].keys()):
                #plt.errorbar(ticks[mark], tri_count[bl], stds[bl])
                plt.plot(ticks[mark], tri_count[mark][bl], label=bl)
        else:
            tot_avg = np.average([tri_count[mark][bl] for bl in tri_count[mark].keys()], axis=0)
            plt.plot(ticks[mark], tot_avg, label=labels[i], alpha=1, linewidth=2.5)

    plt.legend()
    plt.xticks(all_ticks)
    plt.xlabel("Initial depth value[m]")
    plt.ylabel("Number of triangulated features")

    plt.tight_layout()
    if overall:
        plt.savefig(os.path.join(EVO_PATH, "compare", "initdepth", "tri_count_overall"))
    else:
        plt.savefig(os.path.join(EVO_PATH, "compare", "initdepth", "tri_count_"+exps[0][:-2]))


    plt.figure("Compare total errors")
    for i, mark in enumerate(markers):
        if not overall:
            for i, bl in enumerate(errors[mark].keys()):
                #plt.errorbar(ticks[mark], errors[bl], stds[bl])
                plt.plot(ticks[mark], errors[mark][bl], label=bl)
        else:
            tot_avg = np.average([errors[mark][bl] for bl in errors[mark].keys()], axis=0)
            plt.plot(ticks[mark], tot_avg, label=labels[i], alpha=1, linewidth=2.5)

    plt.legend()
    plt.xticks(all_ticks)
    plt.xlabel("Initial depth value[m]")
    plt.ylabel("Total position error[m]")

    plt.tight_layout()

    if overall:
        plt.savefig(os.path.join(EVO_PATH, "compare", "initdepth", "pos_error_overall"))
    else:
        plt.savefig(os.path.join(EVO_PATH, "compare", "initdepth", "pos_error_"+exps[0][:-2]))


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

    avgs = []
    for i, exp in enumerate(exps):
        data = read_traj_data(exp)[bl]

        cnt = 0
        sum = 0
        for run_id in range(len(data)):
            final_pos_val = data[run_id][-1, 1:4]
            pos_error = np.linalg.norm(final_pos_val)
            sum += pos_error
            plt.scatter(f"run {run_id+1}", pos_error, marker="_", c=COLORS[i], s=1000, linewidth=5, alpha=0.8)
            cnt += 1

        avgs.append(sum/cnt)
        labels.append(mpatches.Patch(color=COLORS[i], label=LABELS[i]))

    plt.axvline(len(plt.xticks()[0])-0.5)
    for i in range(len(avgs)):
        plt.scatter(f"Average", avgs[i], marker="_", c=COLORS[i], s=1000, linewidth=5, alpha=0.8)

    plt.legend(handles=labels)
    
    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "runs", "_".join(exps)))


def compare_pos_errors(exps, average=False):
    plt.figure("Compare position errors")

    n = len(exps)
    spread = 0.15*n
    offsets = np.linspace(-spread, spread, len(exps))

    for i, exp in enumerate(exps):
        pos_errors(exp, offset=offsets[i], id=i, average=average)
    
    if average:
        ticks = list(plt.xticks()[0])
        x_val = max(ticks)+2.5
        ticks.append(x_val)
        labels = [str(item) for item in ticks]
        labels[-1] = 'Average'   
    
        plt.xticks(ticks, labels)


    plt.legend(loc='upper right')

    y_min, y_max = plt.ylim()
    plt.ylim(max(0, y_min), min(y_max, 20))

    plt.xlabel("Baseline[cm]")
    plt.ylabel("Final position error [m]")

    plt.tight_layout()
    plt.savefig(os.path.join(EVO_PATH, "compare", "positions", "_".join(exps)))


COLORS = ['b', 'r', 'g', 'c']
LABELS = ['Benchmark', 'Outlier detection', 'Kalman update', 'With init movement, depth=20m']

if __name__ == "__main__":
    try:
        exps = sys.argv[1:-1]

        try:
            height = float(sys.argv[-1])
        except:
            height = 9
            exps.append(sys.argv[-1])

        print(f"Comparing:")
        for exp in exps:
            print("\t", exp)
    except:
        print("Please provide at least one experiment name and a height reference")
        exit()

    #compare_heights(exps, gt=height)
    compare_pos_errors(exps, average=False)
    #compare_triangulation(exps, offset=False)
    compare_each_run(exps, bl='b6')

    #compare_depth_parameter(exps, overall=False, markers=['main25'])



    plt.show()
