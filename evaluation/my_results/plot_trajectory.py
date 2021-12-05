# Plot trajectory and gt!
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import glob

def get_gt(filename):
    # Return nx3 array with gt points
    gt_file = open(filename, "r")
    data = gt_file.readlines()

    gt = np.zeros((len(data), 3))
    for i, line in enumerate(data):
        coordinate = line.split()
        gt[i] = coordinate

    return gt

def get_traj_est(filename):
    # Return nx3 array with positions
    traj_file = open(filename, "r")
    data = traj_file.readlines()

    trajectory = np.zeros((len(data), 3))
    for i, line in enumerate(data):
        if i != 0:
            values = line.split(" ")
            data_point = np.array(values[1:4])
            trajectory[i, :] = data_point

    return trajectory

if __name__ == "__main__":
    files = glob.glob("/home/oveggeland/master_project/rpg_trajectory_evaluation/my_results/office_rounds/*transformed.txt")

    trajectories = []
    for run in files:
        print(run)
        trajectories.append(get_traj_est(run))


    # Plot XY-plane and z-value
    f, axs = plt.subplots(2, 1, gridspec_kw={'height_ratios': [4, 1]})
    axs[0].set_title("XY-plane")

    for i, traj in enumerate(trajectories, start=1):
        axs[0].plot(-traj[:, 0], traj[:, 1], alpha = 0.7, label=f"Run {i}")

        n_values = traj.shape[0]
        print("Number of values is", n_values)

        # Height plotting
        axs[1].plot(-traj[:, 2])
        axs[1].set_title("Z-trajectory")
        axs[1].set_xlabel("Pose number")

    np_traj = np.array(trajectories)
    print(np_traj.shape)

    xy_avg = np.average([1])

    axs[0].legend()


    plt.tight_layout()
    plt.savefig("office_round_trajectories.png")
