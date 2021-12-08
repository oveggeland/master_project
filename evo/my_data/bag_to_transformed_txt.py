#Her skal jeg slå sammen masse kode
import sys
import os
import rosbag
import numpy as np
import math


def quat2dcm(quaternion):
    """Returns direct cosine matrix from quaternion (Hamiltonian, [x y z w])
    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < np.finfo(float).eps * 4.0:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3]),
        (q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3]),
        (q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1])),
        dtype=np.float64)


def dcm2quat(matrix_3x3):
    """Return quaternion (Hamiltonian, [x y z w]) from rotation matrix.
    This algorithm comes from  "Quaternion Calculus and Fast Animation",
    Ken Shoemake, 1987 SIGGRAPH course notes
    (from Eigen)
    """
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix_3x3, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > 0.0:
        t = math.sqrt(t+1.0)
        q[3] = 0.5*t
        t = 0.5/t
        q[0] = (M[2, 1] - M[1, 2])*t
        q[1] = (M[0, 2] - M[2, 0])*t
        q[2] = (M[1, 0] - M[0, 1])*t
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = math.sqrt(M[i, i] - M[j, j] - M[k, k] + 1.0)
        q[i] = 0.5*t
        t = 0.5/t
        q[3] = (M[k, j] - M[j, k])*t
        q[j] = (M[i, j] + M[j, i])*t
        q[k] = (M[k, i] + M[i, k])*t
    return q


def extract_and_transform(bagfile):
    # Setup file to write results
    name = os.path.splitext(bagfile)[0]
    out_file = "".join([name, "_transformed.txt"])
    f = open(out_file, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw\n')

    # Iterate through every line in rosbag and transform it
    n=0
    T = np.eye(4)
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics="/rovio/pose_with_covariance_stamped"):
            quat = np.array([msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

            if (n == 0):
                T[:3, :3] = quat2dcm(quat)
            

            T_old = np.eye(4)
            T_old[0:3, 0:3] = quat2dcm(quat)
            T_old[0:3, 3] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            T_new = T.dot(T_old)

            quat_new = dcm2quat(T_new[:3, :3])
            pose_args = np.concatenate((np.array([msg.header.stamp.to_sec()]), T_new[:3, 3], quat_new))


            f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                    tuple(arg for arg in pose_args))
        
        n += 1


if __name__ == "__main__":
    file_name = sys.argv[1]

    extract_and_transform(file_name)
    print("finished")
