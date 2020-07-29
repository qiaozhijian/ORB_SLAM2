import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab


def set_aspect_equal_3d(ax):
    """
    kudos to https://stackoverflow.com/a/35126679
    :param ax: matplotlib 3D axes object
    """
    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    xmean = np.mean(xlim)
    ymean = np.mean(ylim)
    zmean = np.mean(zlim)

    plot_radius = max([
        abs(lim - mean_)
        for lims, mean_ in ((xlim, xmean), (ylim, ymean), (zlim, zmean))
        for lim in lims
    ])

    ax.set_xlim3d([xmean - plot_radius, xmean + plot_radius])
    ax.set_ylim3d([ymean - plot_radius, ymean + plot_radius])
    ax.set_zlim3d([zmean - plot_radius, zmean + plot_radius])


def getAxis(fig):
    ax = fig.add_subplot("111", projection="3d")
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    return ax


def plot_traj(ax, stamps, traj, style, color, label):
    """
    Plot a trajectory using matplotlib.

    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend

    """
    stamps.sort()
    interval = np.median([s - t for s, t in zip(stamps[1:], stamps[:-1])])
    x = []
    y = []
    z = []
    last = stamps[0]
    # 中间可能有断开的，所以分段画
    for i in range(len(stamps)):
        if stamps[i] - last < 2 * interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
            z.append(traj[i][2])
        elif len(x) > 0:
            ax.plot(x, y, z, style, color=color, label=label)
            label = ""
            x = []
            y = []
            z = []
        last = stamps[i]
    if len(x) > 0:
        ax.plot(x, y, z, style, color=color, label=label)


def plot_slam_eval(second_stamps, est_xyz, first_stamps, gt_xyz):
    fig = plt.figure()

    ax = getAxis(fig)

    plot_traj(ax, first_stamps, gt_xyz, '-', "black", "ground truth")
    plot_traj(ax, second_stamps, est_xyz, '-', "blue", "estimated")

    set_aspect_equal_3d(ax)

    ax.legend(frameon=True)

    plt.show()
