import rosbag
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.collections import LineCollection
from matplotlib import colors as mpcolors
from matplotlib import cm
from matplotlib.ticker import MaxNLocator
import yaml

rcParams = {}
rcParams['axes.titlesize'] = 16
rcParams['axes.labelsize'] = 14
rcParams['xtick.labelsize'] = 10
rcParams['ytick.labelsize'] = 10
rcParams['legend.fontsize'] = 12

scale = 2
scale2 = 1.8
plt.rcParams['axes.titlesize'] = scale * rcParams['axes.titlesize'] 
plt.rcParams['axes.labelsize'] = scale * rcParams['axes.labelsize']
plt.rcParams['xtick.labelsize'] = scale2 * rcParams['xtick.labelsize']
plt.rcParams['ytick.labelsize'] = scale2 * rcParams['ytick.labelsize']
plt.rcParams['legend.fontsize'] = scale * rcParams['legend.fontsize']

def unit_scale(x):
    return (x - x.min()) / (x.max() - x.min())

def getTF(rot: np.ndarray, pos: np.ndarray) -> np.ndarray:
    m = np.eye(4)

    m[:3, :3] = rot
    m[:3, 3] = pos

    return m

def getCameraRot(pos: np.ndarray, lookAt: np.ndarray, up: np.ndarray) -> np.ndarray:

    backward = (pos - lookAt)
    backward /= np.linalg.norm(backward)

    up /= np.linalg.norm(up)

    right = np.cross(up, backward)
    right /= np.linalg.norm(right)

    up = np.cross(backward, right)

    return np.column_stack((backward, right, up))

def make_homogenous(arr: np.ndarray) -> np.ndarray:
    return np.vstack((arr, np.ones((1, arr.shape[1]))))

def setAxesScaleEqual(ax):
    lims = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    centers = np.mean(lims, axis=1)
    half_range = np.max(np.ptp(lims, axis=1)) / 2

    ax.set_xlim3d([centers[0] - half_range, centers[0] + half_range])
    ax.set_ylim3d([centers[1] - half_range, centers[1] + half_range])
    ax.set_zlim3d([centers[2] - half_range, centers[2] + half_range])

T_c_w = None
with open(Path("~/transsphenoidal_simulator/ADF/world/world.yaml").expanduser(), "r") as f:
    data = yaml.safe_load(f)
    camname = "main_camera"
    pos = data[camname]["location"]
    campos = np.array((pos["x"], pos["y"], pos["z"]))

    lookat = data[camname]["look at"]
    camlookat = np.array((lookat["x"], lookat["y"], lookat["z"]))

    up = data[camname]["up"]
    camup = np.array((up["x"], up["y"], up["z"]))

    rot = getCameraRot(campos, camlookat, camup)

    T_w_c = getTF(rot, campos)
    T_c_w = np.linalg.inv(T_w_c)

# bag_file = Path('~/Documents/recordings/userstudy/TS_data_user01_2025-10-28-09-15-55.bag').expanduser()
# fOFFSET = 500
# bOFFSET = -1
# bag_file = Path('~/Documents/recordings/userstudy/TS_data_user02_2_2025-10-28-11-36-06.bag').expanduser()
# fOFFSET = 200
# bOFFSET = -300
bag_file = Path('~/Documents/recordings/userstudy/TS_data_user03_rightnose_2025-10-28-15-27-18.bag').expanduser()
fOFFSET = 400
bOFFSET = -400
USER = "user03"

topic_name = '/ambf/env/Endoscope35degreeinREMS/State'

endo_times_ls = []
endo_poses_ls = []
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
            pos = msg.pose.position
            t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
            endo_times_ls.append(t_sec)
            endo_poses_ls.append(np.array([pos.x, pos.y, pos.z]))

topic_name = '/ambf/env/mastoidectomy_drill/State'
drill_times_ls = []
drill_poses_ls = []
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
            pos = msg.pose.position
            t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
            drill_times_ls.append(t_sec)
            drill_poses_ls.append(np.array([pos.x, pos.y, pos.z]))

endo_poses = (T_c_w @ make_homogenous(np.column_stack(endo_poses_ls[fOFFSET:bOFFSET])))[:3]
drill_poses = (T_c_w @ make_homogenous(np.column_stack(drill_poses_ls[fOFFSET:bOFFSET])))[:3]
endo_times = np.array(endo_times_ls[fOFFSET:bOFFSET])
drill_times = np.array(drill_times_ls[fOFFSET:bOFFSET])


fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(16, 9))

endo_points = endo_poses.T.reshape(-1,1,3)
drill_points = drill_poses.T.reshape(-1,1,3)

endo_segs = np.concatenate([endo_points[:-1], endo_points[1:]], axis=1)
drill_segs = np.concatenate([drill_points[:-1], drill_points[1:]], axis=1)

endo_lc = Line3DCollection(endo_segs,color='#1f77b4', label="Endoscope")
drill_lc = Line3DCollection(drill_segs,color='#ff7f0e', label="Drill")

ax.add_collection3d(endo_lc)
ax.add_collection3d(drill_lc)

ax.set_title("Tool Trajectories")
ax.set_xlabel('X (m)', labelpad=50)
ax.set_ylabel('Y (m)', labelpad=20)
ax.set_zlabel('Z (m)', labelpad=45)

ax.set_box_aspect([1,1,1])

merged = np.hstack((endo_poses, drill_poses))
ax.auto_scale_xyz(merged[0], merged[1], merged[2])

setAxesScaleEqual(ax)
ax.view_init(elev=30, azim=-15)

ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
ax.set_xticklabels([f"{x:.3f}" for x in ax.get_xticks()], rotation=0, ha='right')
ax.set_yticklabels([f"{x:.3f}" for x in ax.get_yticks()], rotation=-0, ha='left')
ax.set_zticklabels([f"{x:.3f}" for x in ax.get_zticks()], rotation=0, ha='left')
plt.legend(ncol=2)

plt.savefig(USER + "_traj.pdf")
plt.show()

##########################

fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(16, 9))

np.where(endo_times - endo_times.min() > 40)
lbound, rbound = (0, 2000)

endo_points = endo_poses[:, lbound:rbound].T.reshape(-1,1,3)
drill_points = drill_poses[:, lbound:rbound].T.reshape(-1,1,3)

endo_segs = np.concatenate([endo_points[:-1], endo_points[1:]], axis=1)
drill_segs = np.concatenate([drill_points[:-1], drill_points[1:]], axis=1)

endo_lc = Line3DCollection(endo_segs,color='#1f77b4', label="Endoscope")
drill_lc = Line3DCollection(drill_segs,color='#ff7f0e', label="Drill")

ax.add_collection3d(endo_lc)
ax.add_collection3d(drill_lc)

ax.set_title("Tool Trajectories (Entry)")
ax.set_xlabel('X (m)', labelpad=50)
ax.set_ylabel('Y (m)', labelpad=20)
ax.set_zlabel('Z (m)', labelpad=45)
ax.set_box_aspect([1,1,1])

merged = np.hstack((endo_poses[:, lbound:rbound], drill_poses[:, lbound:rbound]))
ax.auto_scale_xyz(merged[0], merged[1], merged[2])

setAxesScaleEqual(ax)
ax.view_init(elev=30, azim=-15)

ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
ax.set_xticklabels([f"{x:.3f}" for x in ax.get_xticks()], rotation=0, ha='right')
ax.set_yticklabels([f"{x:.3f}" for x in ax.get_yticks()], rotation=-0, ha='left')
ax.set_zticklabels([f"{x:.3f}" for x in ax.get_zticks()], rotation=0, ha='left')
plt.legend(ncol=2)

plt.savefig(USER + "_traj_entry.pdf")

plt.show()

##########################

fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(16, 9))

np.where(endo_times - endo_times.min() > 40)
lbound, rbound = (2000, endo_times.shape[0])

endo_points = endo_poses[:, lbound:rbound].T.reshape(-1,1,3)
drill_points = drill_poses[:, lbound:rbound].T.reshape(-1,1,3)

endo_segs = np.concatenate([endo_points[:-1], endo_points[1:]], axis=1)
drill_segs = np.concatenate([drill_points[:-1], drill_points[1:]], axis=1)

endo_lc = Line3DCollection(endo_segs,color='#1f77b4', label="Endoscope")
drill_lc = Line3DCollection(drill_segs,color='#ff7f0e', label="Drill")

ax.add_collection3d(endo_lc)
ax.add_collection3d(drill_lc)

ax.set_title("Tool Trajectories (Drilling)")
ax.set_xlabel('X (m)', labelpad=50)
ax.set_ylabel('Y (m)', labelpad=20)
ax.set_zlabel('Z (m)', labelpad=45)
ax.set_box_aspect([1,1,1])

merged = np.hstack((endo_poses[:, lbound:rbound], drill_poses[:, lbound:rbound]))
ax.auto_scale_xyz(merged[0], merged[1], merged[2])

setAxesScaleEqual(ax)
ax.view_init(elev=30, azim=-15)

ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
ax.set_xticklabels([f"{x:.3f}" for x in ax.get_xticks()], rotation=0, ha='right')
ax.set_yticklabels([f"{x:.3f}" for x in ax.get_yticks()], rotation=-0, ha='left')
ax.set_zticklabels([f"{x:.3f}" for x in ax.get_zticks()], rotation=0, ha='left')
plt.legend(ncol=2)

plt.savefig(USER + "_traj_drilling.pdf")

plt.show()