import sys
import numpy
import argparse
import os
from util.associate import *
from util.plot_self import *
from util.metric import *
from util.paras import *


def main(args):
    # offset 为est_traj落后gt_traj的时间，单位s
    matches = associate(args.gt_traj, args.est_traj, float(args.offset), float(args.max_difference))
    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between groundtruth and est_path trajectory! Did you choose the correct sequence?")
    # 取出xyz值
    gt_stamps = numpy.asarray([float(a) for a, b in matches]).transpose()
    gt_xyz = numpy.asarray([[float(value) for value in args.gt_traj[a][0:3]] for a, b in matches])
    gt_quat = numpy.asarray([[float(value) for value in args.gt_traj[a][3:]] for a, b in matches])

    est_stamps = numpy.asarray([float(b) for a, b in matches]).transpose()
    est_xyz = numpy.asarray([[float(value) * float(args.scale) for value in args.est_traj[b][0:3]] for a, b in matches])
    est_quat = numpy.asarray([[float(value) for value in args.est_traj[b][3:]] for a, b in matches])

    # plot_slam_eval(est_stamps, est_xyz, gt_stamps, gt_xyz)

    trans_err_list, rot_err_list = evo_ape(est_xyz, est_quat,gt_xyz,gt_quat)
    # plot_seq(trans_err_list)
    trans_err_mean, trans_err_max, trans_err_median, rot_err_mean, rot_err_max, rot_err_median = evo_statics(
        trans_err_list, rot_err_list)

    # print("trans error max: ", trans_err_max)
    # print("trans error mean: ", trans_err_mean)
    # print("trans error median: ", trans_err_median)
    # print("rot error max: ", rot_err_max)
    # print("rot error mean: ", rot_err_mean)
    # print("rot error median: ", rot_err_median)

    print(trans_err_max)
    print(trans_err_mean)
    print(trans_err_median)
    print(rot_err_max)
    print(rot_err_mean)
    print(rot_err_median)


if __name__ == "__main__":
    # parse command line
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt_path', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)',
                        default="")
    parser.add_argument('--est_path', help='est_path trajectory (format: timestamp tx ty tz qx qy qz qw)', default="")
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)', default=1.0)
    parser.add_argument('--seq', type=str, default="01")
    parser.add_argument('--slam', type=str, default='orb')
    parser.add_argument('--type_slam', type=str, default='slam')
    parser.add_argument('--suffix', type=str, default="")
    parser.add_argument('--scaleAlign', type=bool, default=False)
    parser.add_argument('--Align', type=bool, default=False)
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',
                        default=0.00)
    parser.add_argument('--max_difference',
                        help='maximally allowed time difference for matching entries (default: 0.02 s)',
                        default=0.02)
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)',
                        default="re.pdf")
    parser.add_argument('--verbose',
                        help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)',
                        action='store_true')
    parser.add_argument('--odometry',
                        help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)',
                        action='store_true')
    args = parser.parse_args()

    if args.scaleAlign:
        args.suffix = args.suffix + " -s"
    if args.Align:
        args.suffix = args.suffix + " -va"

    path = '/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/{}'.format(args.seq)
    if args.gt_path == "":
        args.gt_path = os.path.join(path, "vicon_{}.txt".format(args.seq))
    if args.odometry:
        args.est_path = os.path.join(path, "odometry.txt")
    elif args.est_path == "":
        args.est_path = os.path.join(path, "robot{}_{}_stereo_{}.txt").format(args.seq, args.slam, args.type_slam)

    args.gt_traj = np.loadtxt(args.gt_path)
    args.est_traj = np.loadtxt(args.est_path)

    if not args.odometry:
        args.est_traj[:, 1:4] = args.est_traj[:, 1:4] * args.scale
        args.est_traj = trans_robot_slam(args.est_traj)
        eval_name = args.est_path[:-4] + "_BodyFrame.txt"
        saveTum(eval_name, args.est_traj)
    else:
        args.est_traj = trans_robot_odometry(args.est_traj)
        # eval_name = args.est_path[:-4] + "_BodyFrame.txt"
        # saveTum(eval_name, args.est_traj)

    args.gt_traj = trans_robot_vicon(args.gt_traj)
    gt_file_new = args.gt_path[:-4] + "_BodyFrame.txt"
    saveTum(gt_file_new, args.gt_traj)

    args.est_traj = npToDict(args.est_traj)
    args.gt_traj = npToDict(args.gt_traj)

    main(args)

    if (False):
        os.system(
            "evo_traj tum " + eval_name + " --ref " + gt_file_new + " --save_plot {}_{}_{}.pgf --plot --plot_mode=xyz".format(
                args.seq, args.slam, args.type_slam) + args.suffix)
        # angle_deg
        os.system("evo_ape tum " + gt_file_new + " " + eval_name + " -r trans_part -p" + args.suffix)
        os.system("evo_ape tum " + gt_file_new + " " + eval_name + " -r angle_deg" + args.suffix)
        # os.system("evo_rpe tum "+ gt_file_new+ " " + eval_name + args.suffix)
