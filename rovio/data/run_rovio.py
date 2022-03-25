# Script to run a lot of rovio data at once
from email.mime import base
import sys
import os
import glob

DATA_PATH = os.path.dirname(os.path.realpath(__file__))
EVO_PATH = "/home/oveggeland/master_project/evo/bags"

def run_folder_data(rel_dir):
    # Create evo folder if it does not exist
    evo_path = os.path.join(EVO_PATH, rel_dir)
    if not os.path.isdir(evo_path):
        print(f"{evo_path} is not a path, creating new")
        os.mkdir(evo_path)
    # This works!
    
    # Run all .bag files!
    dir_str = rel_dir.split("/")
    if len(dir_str) > 1:
        baseline = dir_str[1]

        bag_files = glob.glob(os.path.join(rel_dir, "*.bag"))
        for bag in bag_files:
            bag_name = os.path.split(bag)[-1]
            if not os.path.isdir(os.path.join(rel_dir, "rovio")):
                os.mkdir(os.path.join(rel_dir, "rovio"))
            os.system(f'roslaunch rovio my_rovio.launch baseline:={baseline} rovio_bag:={bag_name} folder:={rel_dir} record:=true')


    # Recursively continue into subfolders
    sub_folders = [name for name in os.listdir(rel_dir) if os.path.isdir(os.path.join(rel_dir, name))]
    for folder in sub_folders:
        if folder != "rovio":
            new_rel_dir = os.path.join(rel_dir, folder)
            run_folder_data(new_rel_dir)


if __name__ == "__main__":
    print("Running rovio data")
    os.chdir(DATA_PATH)

    if sys.argv[1] == 'all':
        experiments = [name for name in os.listdir(".") if os.path.isdir(name)]
    else:
        experiments = sys.argv[1:]
    
    for exp in experiments:
        run_folder_data(exp)
