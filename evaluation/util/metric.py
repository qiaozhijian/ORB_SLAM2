import numpy as np
import math
from scipy.spatial.transform import Rotation


def saveTum(file_name, traj):
    w = open(file_name, 'w')
    for i in range(traj.shape[0]):
        ss = str(traj[i, 0])
        for j in range(1, 8):
            ss = ss + " " + str(traj[i, j])
        w.write(ss + '\n')
    w.close()


def displacement(x, y):
    return np.sqrt(np.sum(np.square(x - y)))


def rotation(q1, q2):
    rotation = Rotation.from_quat(q1)
    r1 = rotation.as_dcm()
    rotation = Rotation.from_quat(q2)
    r2 = rotation.as_dcm()

    r = np.dot(np.linalg.inv(r1), r2)

    euler = Rotation.from_dcm(r).as_euler("zyx") / math.pi * 180.0

    return np.linalg.norm(euler)


def tum2kitti(t, q):
    rotation = Rotation.from_quat(q)
    r = rotation.as_dcm()
    T = np.eye(4)
    T[0:3, 0:3] = r
    T[0:3, 3] = t
    return T

def tum2kitti2(tum):
    t= tum[:3]
    q= tum[3:]
    rotation = Rotation.from_quat(q)
    r = rotation.as_dcm()
    T = np.eye(4)
    T[0:3, 0:3] = r
    T[0:3, 3] = t
    return T

def flat(l):
    for k in l:
        if not isinstance(k, (list, tuple)):
            yield k
        else:
            yield from flat(k)


def kitti2tum(kitti):
    dcm = kitti[:3, :3]
    t = kitti[:3, 3].tolist()
    q = Rotation.from_dcm(dcm).as_quat()
    tum = list(flat([t, q.tolist()]))
    return np.asarray(tum)


def tum_err(t1, q1, t2, q2):
    T1 = tum2kitti(t1, q1)
    T2 = tum2kitti(t2, q2)
    T = np.dot(np.linalg.inv(T1), T2)
    tum = kitti2tum(T)
    return tum[:3], tum[3:]


def se_err(pose1, pose2):
    T1 = tum2kitti(pose1)
    T2 = tum2kitti(pose2)

    deltaT = np.dot(np.linalg.inv(T1), T2)
    deltaTum = kitti2tum(deltaT)

    t = deltaTum[:3]
    q = deltaTum[3:]
    euler = Rotation.from_quat(q).as_euler("zyx") / math.pi * 180.0

    return t, euler


def quat2euler(quat):
    return Rotation.from_quat(quat).as_euler("zyx") / math.pi * 180.0

def tum2simple(tum):
    return tum[:3], quat2euler(tum[3:])

def evo_rpe(est_xyz, est_quat, gt_xyz, gt_quat):
    if est_xyz.shape[0] < est_xyz.shape[1]:
        est_xyz = est_xyz.transpose()
    if gt_xyz.shape[0] < gt_xyz.shape[1]:
        gt_xyz = gt_xyz.transpose()

    traj_len = est_xyz.shape[0]
    for i in range(1, traj_len):
        print(i)
        t_err, q_err = tum_err(est_xyz[i-1], est_quat[i-1], est_xyz[i], est_quat[i])
        print(t_err,quat2euler(q_err))
        t_err, q_err = tum_err(gt_xyz[i-1], gt_quat[i-1], gt_xyz[i], gt_quat[i])
        print(t_err,quat2euler(q_err))


def evo_ape(est_xyz, est_quat, gt_xyz, gt_quat):
    if est_xyz.shape[0] < est_xyz.shape[1]:
        est_xyz = est_xyz.transpose()
    if gt_xyz.shape[0] < gt_xyz.shape[1]:
        gt_xyz = gt_xyz.transpose()

    traj_len = est_xyz.shape[0]
    trans_route = [0.0]
    rot_route = [0.0]
    trans_err_list = [0.0]
    rot_err_list = [0.0]
    for i in range(1, traj_len):
        trans_route.append(trans_route[i - 1] + displacement(est_xyz[i], est_xyz[i - 1]))
        rot_route.append(rot_route[i - 1] + rotation(est_quat[i], gt_quat[i - 1]))

        t_err, q_err = tum_err(est_xyz[i], est_quat[i], gt_xyz[i], gt_quat[i])
        # 走一段路程后再开始运算
        if trans_route[i] > 1.0:
            trans_err_list.append(np.linalg.norm(t_err) / trans_route[i] * 100)
        if rot_route[i] > 10.0:
            rot_err_list.append(np.linalg.norm(quat2euler(q_err)) / rot_route[i] * 100)

    trans_err_list.sort()
    rot_err_list.sort()

    return trans_err_list, rot_err_list

def evo_statics(trans_err_list, rot_err_list):
    trans_err_mean = np.mean(trans_err_list)
    trans_err_max = np.max(trans_err_list)
    trans_err_median = np.median(trans_err_list)

    rot_err_mean = np.mean(rot_err_list)
    rot_err_max = np.max(rot_err_list)
    rot_err_median = np.median(rot_err_list)

    return trans_err_mean, trans_err_max, trans_err_median, rot_err_mean, rot_err_max, rot_err_median
