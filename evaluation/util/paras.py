import numpy as np
from util.metric import *

T_c0_robot = np.asarray([0.9998199476005879, 0.01896351998860963, 0.0006762319126025606, 0.04216179343179696,
                         0.0002432940438150989, 0.02282305763279319, -0.9997394904915473, 0.08578733170877284,
                         -0.01897401349125333, 0.9995596495206938, 0.02281433457504149, -0.06909713473004092,
                         0, 0, 0, 1]).reshape(4, 4)
# T_imu_c0 = np.asarray([-0.01404787, -0.00799913, -0.99986933, -0.01712035, 0.9998648, 0.00843394, -0.01411528, 0.10571385, 0.00854575,
#      -0.99993244, 0.00787957, 0.11432723, 0., 0., 0., 1.]).reshape(4, 4)
T_imu_c0 = np.asarray([0.0, -0.0, -1, -0.005, 1, 0, 0.0, -0.06, 0, -1, 0.0, 0.13, 0, 0, 0, 1]).reshape(4, 4)

T_odo_imu = np.asarray([-1, 0, 0, 0.133, 0, -1, 0, 0.0, 0, 0, 1, 0.02564, 0., 0., 0., 1.]).reshape(4, 4)

T_robot_c0 = np.linalg.inv(T_c0_robot)
T_odo_robot = np.dot(T_odo_imu, np.dot(T_imu_c0, T_c0_robot))
T_robot_odo = np.linalg.inv(T_odo_robot)

T_odo_c0 = np.dot(T_odo_imu, T_imu_c0)
T_c0_odo = np.linalg.inv(T_odo_c0)
# print(T_odo_c0.reshape(-1))
# print(T_c0_odo.reshape(-1))


def trans_robot_slam(slamTraj):
    num, _ = slamTraj.shape
    for i in range(num):
        T = tum2kitti2(slamTraj[i, 1:])
        T = np.dot(T_robot_c0, T)
        T = np.dot(T, T_c0_robot)
        slamTraj[i, 1:] = np.asarray(kitti2tum(T))
    return slamTraj


def trans_robot_vicon(viconTraj):
    T_vicon_robot = tum2kitti2(viconTraj[0, 1:])
    T_robot_vicon = np.linalg.inv(T_vicon_robot)
    # print(T_vicon_robot[:3,:3])
    # print(T_robot_vicon[:3,:3])
    # 返回欧拉角也是zyx顺序
    # print(Rotation.from_dcm(T_robot_vicon[:3,:3]).as_euler("zyx")/math.pi*180.0)
    # 返回欧拉角也是zyx顺序
    # print(Rotation.from_dcm(T_vicon_robot[:3,:3]).as_euler("zyx")/math.pi*180.0)
    num, _ = viconTraj.shape
    for i in range(num):
        try:
            T = tum2kitti2(viconTraj[i, 1:])
        except ValueError:
            print(viconTraj[i, 1:])
        T_ = np.dot(T_robot_vicon, T)
        # print(T[:3,3],T_[:3,3])
        # 返回欧拉角也是zyx顺序
        # print(Rotation.from_dcm(T[:3,:3]).as_euler("zyx")/math.pi*180.0)
        # T = np.dot(T, T_robot_vicon)
        viconTraj[i, 1:] = np.asarray(kitti2tum(T_))
    return viconTraj


def trans_robot_odometry(odometryTraj):
    num, _ = odometryTraj.shape
    for i in range(num):
        T = tum2kitti2(odometryTraj[i, 1:])
        T = np.dot(T_robot_odo, T)
        T = np.dot(T, T_odo_robot)
        odometryTraj[i, 1:] = np.asarray(kitti2tum(T))

    T_odo_origin = tum2kitti2(odometryTraj[0, 1:])
    T_origin_odo = np.linalg.inv(T_odo_origin)
    for i in range(num):
        try:
            T = tum2kitti2(odometryTraj[i, 1:])
        except ValueError:
            print(odometryTraj[i, 1:])
        T_ = np.dot(T_origin_odo, T)
        odometryTraj[i, 1:] = np.asarray(kitti2tum(T_))

    return odometryTraj


def alignTime(slamTraj, viconTraj):
    timeMax = slamTraj[-1, 0]
    timeMin = slamTraj[0, 0]
    num = viconTraj.shape[0]
    timeVicon = np.linspace(timeMin, timeMax, num)
    viconTraj[:, 0] = timeVicon.reshape(-1)
    return slamTraj, viconTraj


def getUsefulPart(traj, vicon=False):
    trans = traj[:, 1:4]
    num, _ = traj.shape;
    endIdx1 = 0
    if vicon:
        thresold = 0.00002
    else:
        thresold = 0.00055
    for i in range(num):
        egoMotion = np.sqrt(np.sum(np.square(trans[i + 1, :] - trans[i, :])))
        # print(egoMotion)
        if egoMotion < thresold:
            endIdx1 = i
        else:
            break
    traj = traj[endIdx1:]

    trans = traj[:, 1:4]
    num, _ = traj.shape;
    endIdx2 = num - 1
    for i in range(num):
        j = num - i - 1;
        egoMotion = np.sqrt(np.sum(np.square(trans[j - 1, :] - trans[j, :])))
        # print(egoMotion)
        if egoMotion < thresold:
            endIdx2 = j
        else:
            break
    traj = traj[:endIdx2]

    return traj, endIdx1, endIdx2

if __name__ == "__main__":
    T0 = np.eye(3)

    T_o1_o2 = np.asarray([0,-1,0,1,0,0,0,0,1]).reshape(3,3)
    T_c1_c2 = np.asarray([0,-1,-1,1,0,1,0,0,1]).reshape(3,3)
    T_o_c = np.asarray([1,0,1,0,1,0,0,0,1]).reshape(3,3)
    T_c_o = np.linalg.inv(T_o_c)

    print("T_c1_c2",'\n',T_c1_c2)
    T_c1_c2_ = np.dot(np.dot(T_o_c,T_o1_o2),T_c_o)
    print("T_c1_c2_",'\n',T_c1_c2_)
    T_c1_c2_ = np.dot(np.dot(T_c_o,T_o1_o2),T_o_c)
    print("T_c1_c2_",'\n',T_c1_c2_)
