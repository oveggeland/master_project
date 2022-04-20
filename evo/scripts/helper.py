import os
import math
import numpy as np
from numpy import rad2deg

EVO_PATH = "/home/oveggeland/master_project/evo"

BASELINES = {
    'b1': 12.5,
    'b2': 15,
    'b3': 17.5,
    'b4': 20,
    'b5': 22.5,
    'b6': 25
}

FRAMEID = 0
POINTID = 1
CAMID = 2
CAM0ST = 3 
CAM1ST = 4 
DIST = 5 
DCOV = 6 
TRI = 7

PCL_POINT_STEP = 80
PCL_NPOINTS = 25
PCL_LABELS = ["id","camId","rgb","cam0st", "cam1st","x","y","z","b_x","b_y","b_z","d","c_00","c_01","c_02","c_11","c_12","c_22","c_d", "tri"]
PCL_OFFSETS = [0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76]
PCL_DTYPES = ["i", "i", "I", "I", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "i"]
PCL_DSIZE = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]

def fit_poly(x, y, dim):
    model = np.polyfit(x, y, dim)
    p = np.poly1d(model)
    return p(x)

def try_create_path(path):
    if not os.path.isdir(path):
        print(f"{path} is not a path, creating new")
        os.mkdir(path)

def quat_to_euler(quat, deg=True):
    x, y, z, w = tuple(quat)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    if deg:
        return rad2deg(roll_x), rad2deg(pitch_y), rad2deg(yaw_z) # in degrees
    else:
        return roll_x, pitch_y, yaw_z # in radians