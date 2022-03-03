# Script to read all rovio data at once!
import sys
import os
import glob
import json
import struct
import rosbag

EVO_PATH = "/home/oveggeland/master_project/evo"

def try_create_path(path):
    if not os.path.isdir(path):
        print(f"{path} is not a path, creating new")
        os.mkdir(path)

def create_depth_data(bagpath):
    target_file = os.path.join(EVO_PATH, "data", "depths", bagpath[:-4]+".txt")
    f = open(target_file, "w")
    f.truncate(0) # Remove old data

    bag = rosbag.Bag(bagpath)

    # PCL values
    n_points = 25
    point_step = 80
    # [point_id, cam_id, cam0_st, cam1_st, d, d_cov, tri]
    offsets = [0, 4, 12, 16, 44, 72, 76]

    frame_id = 0
    for _, msg, _ in bag.read_messages(topics=["/rovio/pcl"]):
        data = msg.data
        for point in range(n_points):
            point_id = struct.unpack("i", bytes(data[point*point_step+offsets[0]:point*point_step+offsets[0]+4]))[0]
            cam_id = struct.unpack("i", bytes(data[point*point_step+offsets[1]:point*point_step+offsets[1]+4]))[0]
            cam0_st = struct.unpack("I", bytes(data[point*point_step+offsets[2]:point*point_step+offsets[2]+4]))[0]
            cam1_st = struct.unpack("I", bytes(data[point*point_step+offsets[3]:point*point_step+offsets[3]+4]))[0]
            d = struct.unpack("f", bytes(data[point*point_step+offsets[4]:point*point_step+offsets[4]+4]))[0]
            d_cov = struct.unpack("f", bytes(data[point*point_step+offsets[5]:point*point_step+offsets[5]+4]))[0]
            tri = struct.unpack("I", bytes(data[point*point_step+offsets[6]:point*point_step+offsets[6]+4]))[0]

            point_line = [frame_id, point_id, cam_id, cam0_st, cam1_st, d, d_cov, tri]
            target_string = " ".join([str(element) for element in point_line])
            f.write(target_string+"\n")
        
        frame_id += 1

def create_tf_file(bagpath):
    tf_path = os.path.join(EVO_PATH, "data", "tf", bagpath[:-4]+".json")

    bag = rosbag.Bag(bagpath)
    for _, msg, _ in bag.read_messages(topics=["/rovio/pose_with_covariance_stamped"]):
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        tf = {
            'x':pos.x,
            'y':pos.y,
            'z':pos.z,
            'qx':quat.x,
            'qy':quat.y,
            'qz':quat.z,
            'qw':quat.w,
        }

        with open(tf_path, 'w') as outfile:
                json.dump(tf, outfile)

        return tf_path


def create_traj_data(bagpath):
    # First create tf
    tf_path = create_tf_file(bagpath)
    traj_path = os.path.join(EVO_PATH, "data", "trajs", bagpath[:-4]+".txt")

    os.system(f"evo_traj bag {bagpath} /rovio/pose_with_covariance_stamped --transform_left {tf_path} --invert_transform --save_as_tum")
    os.system(f"mv rovio_pose_with_covariance_stamped.tum {traj_path}")
    

    pass

def read_folder_data(rel_dir):
    print("---------", rel_dir, "---------")
    # Create folders they don't exist
    try_create_path(os.path.join(EVO_PATH, "data", "depths", rel_dir))
    try_create_path(os.path.join(EVO_PATH, "data", "trajs", rel_dir))
    try_create_path(os.path.join(EVO_PATH, "data", "tf", rel_dir))

    # Create depth and traj data
    bags = glob.glob(os.path.join(rel_dir, "*.bag"))
    for bag in bags:
        create_depth_data(bag)
        create_traj_data(bag)

    # Recursively continue into subfolders
    sub_folders = [name for name in os.listdir(rel_dir) if os.path.isdir(os.path.join(rel_dir, name))]
    for folder in sub_folders:
        if folder != "rovio":
            new_rel_dir = os.path.join(rel_dir, folder)
            read_folder_data(new_rel_dir)


if __name__ == "__main__":
    print("Reading rovio data")
    os.chdir(os.path.join(EVO_PATH, "bags"))

    if sys.argv[1] == 'all':
        experiments = [name for name in os.listdir(".") if os.path.isdir(name)]
    else:
        experiments = sys.argv[1:]


    for exp in experiments:
        read_folder_data(exp)
