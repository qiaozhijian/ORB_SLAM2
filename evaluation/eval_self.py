import sys
import numpy
import argparse
import associate
from plot_self import plot_slam_eval
from metric import evo_ape

if __name__ == "__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('ground_truth', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('estimated', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',
                        default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)', default=0.9)
    parser.add_argument('--max_difference',
                        help='maximally allowed time difference for matching entries (default: 0.02 s)',
                        default=0.02)
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--verbose',
                        help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)',
                        action='store_true')
    args = parser.parse_args()

    first_list = associate.read_file_list(args.ground_truth, False)
    second_list = associate.read_file_list(args.estimated, False)

    matches = associate.associate(first_list, second_list, float(args.offset), float(args.max_difference))
    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    # 取出xyz值
    gt_stamps = numpy.asarray([float(a) for a, b in matches]).transpose()
    est_stamps = numpy.asarray([float(b) for a, b in matches]).transpose()
    gt_xyz = numpy.asarray([[float(value) for value in first_list[a][0:3]] for a, b in matches])
    est_xyz = numpy.asarray([[float(value) * float(args.scale) for value in second_list[b][0:3]] for a, b in matches])
    gt_quat = numpy.asarray([[float(value) for value in first_list[a][3:]] for a, b in matches])
    est_quat = numpy.asarray([[float(value) for value in second_list[b][3:]] for a, b in matches])

    plot_slam_eval(est_stamps, est_xyz, gt_stamps, gt_xyz)

    trans_err_mean, trans_err_max, trans_err_median, rot_err_mean, rot_err_max, rot_err_median = evo_ape(est_xyz, est_quat, gt_xyz, gt_quat)

    print(trans_err_mean, trans_err_max, trans_err_median, rot_err_mean, rot_err_max, rot_err_median)