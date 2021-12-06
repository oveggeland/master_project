# Her skal det allignes
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




if __name__ == "__main__":
    src_file = "big_rounds/traj_3.txt"

    f = open(src_file, "r")
    lines = f.readlines()

    init_pose = lines[1]
    init_quat = init_pose.split()[4:8]
    print("init quaternion is", init_quat)

    # Create transformation matrix from quat
    rotation = quat2dcm(init_quat)

    T = np.eye(4)
    T[:3, :3] = rotation

    # STOLEN CODE
    timestamp = []
    positions = []
    quats = []
    i = 0
    for x in lines:
        if(i == 0):
            header = x
            i = i+1
            continue
        contents = x.split(' ')
        timestamp.append(contents[0])
        positions.append(np.array(contents[1:4], dtype=float))
        quats.append(np.array(contents[4:8], dtype=float))
    f.close()

    N = len(positions)
    assert(N == len(quats))
    transformed_positions = np.zeros([N, 3])
    transformed_quats = np.zeros([N, 4])
    for i in range(N):
        T_old = np.eye(4)
        T_old[0:3, 0:3] = quat2dcm(quats[i])
        T_old[0:3, 3] = positions[i]
        T_new = T.dot(T_old)
        transformed_positions[i, :] = T_new[0:3, 3]
        transformed_quats[i, :] = dcm2quat(T_new[0:3, 0:3])

    file_lines = []
    file_lines.append(header)
    for i in range(N):
        file_lines.append(''.join([str(timestamp[i]), ' ',
                                   str(transformed_positions[i, 0]), ' ',
                                   str(transformed_positions[i, 1]), ' ',
                                   str(transformed_positions[i, 2]), ' ',
                                   str(transformed_quats[i, 0]), ' ',
                                   str(transformed_quats[i, 1]), ' ',
                                   str(transformed_quats[i, 2]), ' ',
                                   str(transformed_quats[i, 3]), '\n']))

    outfn = src_file[:-4] + "transformed.txt"
    with open(outfn, 'w') as f:
        f.writelines(file_lines)
    print("Wrote transformed poses to file {0}.".format(outfn))
