import os
import math

from numpy import rad2deg

EVO_PATH = "/home/oveggeland/master_project/evo"

BASELINES = {
    'b1': 12,
    'b2': 14.5,
    'b3': 17,
    'b4': 19.5,
    'b5': 22,
}

FRAMEID = 0
POINTID = 1
CAMID = 2
CAM0ST = 3 
CAM1ST = 4 
DIST = 5 
DCOV = 6 
TRI = 7

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