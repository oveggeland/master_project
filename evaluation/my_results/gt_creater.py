# Create dummy ground truth
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Create ground thruth')
parser.add_argument('--pose', type=str, help='Directory of pose file')
parser.add_argument('--gt', type=str, help='Directory of gt file')


def create_gt(poses, gt):
    gt_file = open(gt, "r")
    gt_points = gt_file.readlines()


    pose_file = open(poses, "r")
    gt_file = open("stamped_groundtruth.txt", "w")

    lines = pose_file.readlines()

    point_cnt = 0
    for line in lines:

        line_sep = line.split()
        assert len(line_sep) == 8, "Not 8 entries in line?"
    
        point_index = min(len(gt_points)-1, point_cnt)
        gt_line = ' '.join([line_sep[0], gt_points[point_index]])
        gt_file.write(gt_line)

        point_cnt += 1

    print("finished")

if __name__ == "__main__":
    args = parser.parse_args()
    create_gt(args.pose, args.gt)
