import os
import sys

if __name__ == "__main__":
    print("-------Running full pipeline!-------")
    try:
        exp = sys.argv[1]
    except:
        print("ERROR: Please provide an experiment name")
        exit()
    
    # Read bag files!
    os.system(f"python3 read_bagfiles.py {exp}")
    # Plot experimental depth data!
    os.system(f"python3 plot_experiment_depth_data.py {exp} --dont_show")
    # Read bag files!
    os.system(f"python3 plot_experiment_traj_data.py {exp} --dont_show")
    # Read bag files!
    os.system(f"python3 run_evo.py {exp}")