from matplotlib import cm
from mpl_toolkits import mplot3d
from numpy.core.overrides import set_array_function_like_doc
import rosbag
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from bisect import bisect_left
from scipy.spatial.distance import cdist
import matplotlib.colors as mpcolors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.ticker import MaxNLocator

rcParams = {}
rcParams['axes.titlesize'] = 16
rcParams['axes.labelsize'] = 14
rcParams['xtick.labelsize'] = 10
rcParams['ytick.labelsize'] = 10
rcParams['legend.fontsize'] = 12

scale = 2
plt.rcParams['axes.titlesize'] = scale * rcParams['axes.titlesize'] 
plt.rcParams['axes.labelsize'] = scale * rcParams['axes.labelsize']
plt.rcParams['xtick.labelsize'] = scale * rcParams['xtick.labelsize']
plt.rcParams['ytick.labelsize'] = scale * rcParams['ytick.labelsize']
plt.rcParams['legend.fontsize'] = scale * rcParams['legend.fontsize']

def quaternion_to_rotation_matrix(q):
    q = np.array(q)
    w, x, y, z = q

    n = np.linalg.norm(q)
    if n == 0:
        raise ValueError("zero-norm quaternion")
    q /= n
    w, x, y, z = q

    R = np.array([
        [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,           1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,           2*y*z + 2*x*w,       1 - 2*x**2 - 2*y**2]
    ])
    
    return R

def get_closest_key(keys_sorted, key):
    idx = bisect_left(keys_sorted, key)

    # figure out if key is closest to the one it's inserted before or after
    # also protect against out of bounds index
    key1 = keys_sorted[min(idx, len(keys_sorted) - 1)]
    key2 = keys_sorted[max(idx - 1, 0)]

    dist1 = np.abs(key1 - key)
    dist2 = np.abs(key2 - key)

    if dist1 < dist2:
        return key1, dist1
    else:
        return key2, dist2

def unit_scale(x, by=None):
    if by is None:
        by = x


    return (x - by.min()) / (by.max() - by.min())

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

bag_file = Path('~/Documents/recordings/userstudy/TS_data_user01_2025-10-28-09-15-55.bag').expanduser()
USER = "user01"
# bag_file = Path('~/Documents/recordings/userstudy/TS_data_user02_2_2025-10-28-11-36-06.bag').expanduser()
# bag_file = Path('~/Documents/recordings/userstudy/TS_data_user03_rightnose_2025-10-28-15-27-18.bag').expanduser()
# USER = "user03"

topic_name = '/ambf/env/plugin/volumetric_drilling/endoscope/drill_rbforce_feedback'
force_map = {}
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        f = msg.wrench.force

        t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        force_map[t_sec] = np.linalg.norm((f.x, f.y, f.z))


force_keys_sorted = sorted(force_map)

topic_name = '/ambf/env/Contact_Endoscope/State'
contact_points = {}
contact_forces = {}
force_dists = []
contact_steps = {}
contact_times = {}
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

        force_key, force_dist = get_closest_key(force_keys_sorted, t_sec)
        force = force_map[force_key]

        force_dists.append(force_dist)

        contact_steps[msg.sim_step] = t_sec

        for event in msg.contact_events:
            name = event.object_name.data

            for contact, contact_local in zip(event.contact_data, event.contact_data_local):
                if np.abs(contact.distance.data) > 1e-6:
                    continue
                cp = contact_local.local_point_a
                cp = np.array([[cp.x, cp.y, cp.z]])
                # cp = rot.T @ np.array([[cp.x, cp.y, cp.z]]).T - rot.T @ trans
                contact_points.setdefault(name, []).append(cp)
                contact_forces.setdefault(name, []).append(force)
                contact_times.setdefault(name, []).append(t_sec)

plt.hist(force_dists)
plt.title("force_dists")
plt.show()

fig = plt.figure(figsize=(16, 9))
cp_times_drill = np.array(contact_times["/ambf/env/BODY 4mm"])
cp_times_drill -= cp_times_drill.min()
cp_forces_drill = np.array(contact_forces["/ambf/env/BODY 4mm"])

# insert zeros in between for sparsity
cp_times_drill_stagger = np.sort(np.concatenate([cp_times_drill, cp_times_drill + 1e-9]))
cp_forces_drill_stagger = np.zeros_like(cp_times_drill_stagger)
cp_forces_drill_stagger[::2] = cp_forces_drill

plt.step(cp_times_drill_stagger, cp_forces_drill_stagger, where="post", label="Drill")

cp_forces = None
cp_times = None
try:
    cp_forces = np.array(contact_forces["/ambf/env/BODY NoseGhost"])
    cp_times = np.array(contact_times["/ambf/env/BODY NoseGhost"])

    try:
        cp_forces = np.concatenate((cp_forces, np.array(contact_forces["/ambf/env/BODY Face"])))
        cp_times = np.array(contact_times["/ambf/env/BODY Face"])
    except KeyError:
        print("NO FACE FORCES")
except KeyError:
    print("NO NOSE FORCES")

try:
    cp_forces = np.array(contact_forces["/ambf/env/BODY Face"])
    cp_times = np.array(contact_times["/ambf/env/BODY Face"])

    try:
        cp_forces = np.concatenate((cp_forces, np.array(contact_forces["/ambf/env/BODY NoseGhost"])))
        cp_times = np.array(contact_times["/ambf/env/BODY NoseGhost"])
    except KeyError:
        print("NO NOSE FORCES")
except KeyError:
    print("NO FACE FORCES")


cp_times -= cp_times.min()

cp_forces = cp_forces[np.argsort(cp_times)]
cp_times = np.sort(cp_times)

cp_times_stagger = np.sort(np.concatenate([cp_times, cp_times + 1e-9]))
cp_forces_stagger = np.zeros_like(cp_times_stagger)
cp_forces_stagger[::2] = cp_forces


plt.step(cp_times_stagger, cp_forces_stagger, where="post", label="Anatomy")

plt.xlabel('Time (s)')
plt.ylabel('Force (N)')

plt.title('Endoscope Contact Force vs. Time')
plt.legend()

plt.savefig(USER + "_contact_force_time.pdf")
plt.show()