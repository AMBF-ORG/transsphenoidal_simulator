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

def load_obj(filename):
    vertices = []
    faces = []
    with open(filename) as f:
        for line in f:
            if line.startswith('v '):  # Vertex
                parts = line.strip().split()
                vertices.append([float(x) for x in parts[1:4]])
            elif line.startswith('f '):  # Face
                parts = line.strip().split()
                # Handle faces like 'f 1 2 3' or 'f 1/1 2/2 3/3'
                face = [int(p.split('/')[0]) - 1 for p in parts[1:]]
                faces.append(face)
    return np.array(vertices), faces

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


def generate_capsule_mesh(radius, height, lin_offset, rsteps, hsteps):
    trunk_length = height - radius * 2
    caps_dist = height / 2.0 - radius

    # cylinder
    theta = np.linspace(0, 2 * np.pi, rsteps)
    x_cyl = np.linspace(-caps_dist, caps_dist, hsteps)
    theta, x_cyl = np.meshgrid(theta, x_cyl)

    y_cyl = radius * np.cos(theta)
    z_cyl = radius * np.sin(theta)

    phi = np.linspace(0, np.pi, rsteps)
    theta_sph = np.linspace(0, 2 * np.pi, rsteps)
    theta_sph, phi = np.meshgrid(theta_sph, phi)

    # top hemisphere
    x_top = radius * np.cos(phi) + caps_dist
    y_top = radius * np.sin(phi) * np.sin(theta_sph)
    z_top = radius * np.sin(phi) * np.cos(theta_sph)

    # bottom hemisphere
    x_bot = radius * np.cos(phi) - caps_dist
    y_bot = radius * np.sin(phi) * np.sin(theta_sph)
    z_bot = radius * np.sin(phi) * np.cos(theta_sph)

    x_cyl += lin_offset[0]
    x_top += lin_offset[0]
    x_bot += lin_offset[0]

    y_cyl += lin_offset[1]
    y_top += lin_offset[1]
    y_bot += lin_offset[1]

    z_cyl += lin_offset[2]
    z_top += lin_offset[2]
    z_bot += lin_offset[2]

    return (x_cyl, y_cyl, z_cyl), (x_top, y_top, z_top), (x_bot, y_bot, z_bot)


def compute_surface_forces(surf_points, contact_points, contact_forces, sigma):
    dists = cdist(surf_points, contact_points)

    weights = np.exp(-(dists**2) / (2 * sigma**2))

    # normalizing doesn't perform well because it distorts the distance information, i.e. small weight among all 0s becomes full contribution
    # removing small weights using cutoff has some problems where if too large, the distance can cross the cylinder, i.e. euclidean line instead of geodesic
    # sigma has similar issues, but 1e-3 works well

    # weights[dists > cutoff] = 0
    # print(np.exp(-(cutoff**2) / (2 * sigma**2)))
    # print("weights", weights.min(), weights.max())
    # print("dists", dists.min(), dists.max())
    # norm_denom = weights.sum(axis=1, keepdims=True)
    # print("norm denom", norm_denom.min(), norm_denom.max())
    # weights_normed = np.divide(
    #     weights, norm_denom, out=np.zeros_like(weights), where=norm_denom != 0
    # )

    # use max instead of summing or taking an average to preserve scale
    # like a gaussian attenuated nearest point
    # or rescale directly to contact_force range

    surf_forces = (weights * contact_forces).max(axis=1)
    # surf_forces = weights @ contact_forces
    # surf_forces = unit_scale(surf_forces) * contact_forces.max()

    return surf_forces

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

topic_name = '/ambf/env/plugin/volumetric_drilling/endoscope/drill_rbforce_feedback'
force_map = {}
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        f = msg.wrench.force

        t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        force_map[t_sec] = np.linalg.norm((f.x, f.y, f.z))

topic_name = '/ambf/env/Endoscope35degreeinREMS/State'
pose_map = {}
pose_steps = {}
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        pos = msg.pose.position
        ori = msg.pose.orientation
        trans = np.array([[pos.x, pos.y, pos.z]]).T
        rot = quaternion_to_rotation_matrix([ori.x, ori.y, ori.z, ori.w])

        t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        pose_map[t_sec] = (rot, trans)

        pose_steps[msg.sim_step] = t_sec



pose_keys_sorted = sorted(pose_map)
force_keys_sorted = sorted(force_map)

topic_name = '/ambf/env/Contact_Endoscope/State'
contact_points = {}
contact_forces = {}
pose_dists = []
force_dists = []
contact_steps = {}
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        t_sec = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        pose_key, pose_dist = get_closest_key(pose_keys_sorted, t_sec)
        rot, trans = pose_map[pose_key]

        force_key, force_dist = get_closest_key(force_keys_sorted, t_sec)
        force = force_map[force_key]

        pose_dists.append(pose_dist)
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

# merge across the different bodies being interacted with
for k, v in contact_points.items():
    contact_points[k] = np.vstack(v)

plt.hist(pose_dists)
plt.title("pose_dists")
plt.show()

plt.hist(force_dists)
plt.title("force_dists")
plt.show()

(x_cyl, y_cyl, z_cyl), (x_top, y_top, z_top), (x_bot, y_bot, z_bot) = (
    generate_capsule_mesh(
        radius=0.0021, height=0.2, lin_offset=(0.1, 0, 0), rsteps=50, hsteps=500
    )
)

mesh_points = np.vstack(
    [
        np.column_stack([x_cyl.ravel(), y_cyl.ravel(), z_cyl.ravel()]),
        np.column_stack([x_top.ravel(), y_top.ravel(), z_top.ravel()]),
        np.column_stack([x_bot.ravel(), y_bot.ravel(), z_bot.ravel()]),
    ]
)


vertices, faces = load_obj("ADF/meshes/high_res/EndoscopeNoShaftdVRK.OBJ")
# vertices = (np.array([
#     [1,  0,  0],
#     [0,  0, 1],
#     [0,  -1,  0]
# ]) @ vertices.T).T
mesh = [vertices[face] for face in faces]
cps_all = np.vstack(list(contact_points.values()))
cp_forces_all = np.concatenate(list(contact_forces.values()))

surf_forces_all = compute_surface_forces(mesh_points, cps_all, cp_forces_all, sigma=1e-3)

cps = contact_points["/ambf/env/BODY 4mm"]
cp_forces = np.array(contact_forces["/ambf/env/BODY 4mm"])

surf_forces = compute_surface_forces(mesh_points, cps, cp_forces, sigma=1e-3)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(16, 9))
forces_cyl, forces_top, forces_bot = np.split(unit_scale(surf_forces, surf_forces_all), (np.prod(x_cyl.shape), np.prod(x_cyl.shape) + np.prod(x_top.shape)))

surf_cyl = ax.plot_surface(x_cyl, y_cyl, z_cyl, rstride=1, cstride=1, facecolors=cm.jet(forces_cyl.reshape(x_cyl.shape)))
surf_top = ax.plot_surface(x_top, y_top, z_top, rstride=1, cstride=1, facecolors=cm.jet(forces_top.reshape(x_top.shape)))
surf_bot = ax.plot_surface(x_bot, y_bot, z_bot, rstride=1, cstride=1, facecolors=cm.jet(forces_bot.reshape(x_bot.shape)))

# sc = ax.scatter(cps[:, 0], cps[:, 1], cps[:, 2], c=cp_forces, cmap='viridis')

mesh = [vertices[face] for face in faces]

collection = Poly3DCollection(mesh, alpha=0.8, edgecolor='none')
collection.set_facecolor((0.279, 0.184, 0.319, 1.0))

ax.add_collection3d(collection)

sm = cm.ScalarMappable(norm=mpcolors.Normalize(vmin=surf_forces_all.min(), vmax=surf_forces_all.max()), cmap=cm.jet)

cbar = fig.colorbar(sm, ax=ax, pad=0.1)  # pad adjusts spacing
cbar.set_label('Force (N)', labelpad=15)

ax.set_xlabel('X (m)', labelpad=35)
ax.set_ylabel('Y (m)', labelpad=35)
ax.set_zlabel('Z (m)', labelpad=45)
ax.set_title('Endoscope Contact Force from Drill')
ax.set_box_aspect([1,1,1])
setAxesScaleEqual(ax)
ax.view_init(elev=30, azim=-135)

ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
ax.set_xticklabels([f"{x:.3f}" for x in ax.get_xticks()], rotation=0, ha='left')
ax.set_yticklabels([f"{x:.3f}" for x in ax.get_yticks()], rotation=-0, ha='right')
ax.set_zticklabels([f"{x:.3f}" for x in ax.get_zticks()], rotation=0, ha='right')

plt.savefig(USER + "_contact_force_drill.pdf")
plt.show()

############

try:
    cps = contact_points["/ambf/env/BODY NoseGhost"]
    cp_forces = np.array(contact_forces["/ambf/env/BODY NoseGhost"])

    try:
        cps = np.vstack((cps, contact_points["/ambf/env/BODY Face"]))
        cp_forces = np.concatenate((cp_forces, np.array(contact_forces["/ambf/env/BODY Face"])))
    except KeyError:
        print("NO FACE FORCES")
except KeyError:
    print("NO NOSE FORCES")

try:
    cps = contact_points["/ambf/env/BODY Face"]
    cp_forces = np.array(contact_forces["/ambf/env/BODY Face"])

    try:
        cps = np.vstack((cps, contact_points["/ambf/env/BODY NoseGhost"]))
        cp_forces = np.concatenate((cp_forces, np.array(contact_forces["/ambf/env/BODY NoseGhost"])))
    except KeyError:
        print("NO NOSE FORCES")
except KeyError:
    print("NO FACE FORCES")


surf_forces = compute_surface_forces(mesh_points, cps, cp_forces, sigma=1e-3)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(16, 9))

forces_cyl, forces_top, forces_bot = np.split(unit_scale(surf_forces, surf_forces_all), (np.prod(x_cyl.shape), np.prod(x_cyl.shape) + np.prod(x_top.shape)))

surf_cyl = ax.plot_surface(x_cyl, y_cyl, z_cyl, rstride=1, cstride=1, facecolors=cm.jet(forces_cyl.reshape(x_cyl.shape)))
surf_top = ax.plot_surface(x_top, y_top, z_top, rstride=1, cstride=1, facecolors=cm.jet(forces_top.reshape(x_top.shape)))
surf_bot = ax.plot_surface(x_bot, y_bot, z_bot, rstride=1, cstride=1, facecolors=cm.jet(forces_bot.reshape(x_bot.shape)))

# sc = ax.scatter(cps[:, 0], cps[:, 1], cps[:, 2], c=cp_forces, cmap='viridis')

mesh = [vertices[face] for face in faces]

collection = Poly3DCollection(mesh, alpha=0.8, edgecolor='none')
collection.set_facecolor((0.279, 0.184, 0.319, 1.0))

ax.add_collection3d(collection)

sm = cm.ScalarMappable(norm=mpcolors.Normalize(vmin=surf_forces_all.min(), vmax=surf_forces_all.max()), cmap=cm.jet)

cbar = fig.colorbar(sm, ax=ax, pad=0.1)  # pad adjusts spacing
cbar.set_label('Force (N)', labelpad=15)

ax.set_xlabel('X (m)', labelpad=35)
ax.set_ylabel('Y (m)', labelpad=35)
ax.set_zlabel('Z (m)', labelpad=45)
ax.set_title('Endoscope Contact Force from Anatomy')
ax.set_box_aspect([1,1,1])
setAxesScaleEqual(ax)
ax.view_init(elev=30, azim=-135)

ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
ax.set_xticklabels([f"{x:.3f}" for x in ax.get_xticks()], rotation=0, ha='left')
ax.set_yticklabels([f"{x:.3f}" for x in ax.get_yticks()], rotation=-0, ha='right')
ax.set_zticklabels([f"{x:.3f}" for x in ax.get_zticks()], rotation=0, ha='right')

plt.savefig(USER + "_contact_force_anatomy.pdf")

plt.show()
#####################################################################################################

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # for label, cps in contact_points.items():
# #     print(label)
# #     ax.scatter(cps[:, 0], cps[:, 1], cps[:, 2], label=label)

# # Define origin of the frame
# origin = np.array([0, 0, 0])

# # Define unit vectors for axes
# x_axis = np.array([1, 0, 0])
# y_axis = np.array([0, 1, 0])
# z_axis = np.array([0, 0, 1])

# # Plot coordinate frame using quiver
# ax.quiver(*origin, *x_axis, length=0.5, color='r', label='X')
# ax.quiver(*origin, *y_axis, length=0.5, color='g', label='Y')
# ax.quiver(*origin, *z_axis, length=0.5, color='b', label='Z')

# cps_face = contact_points["/ambf/env/BODY Face"]
# cp_forces_face = contact_forces["/ambf/env/BODY Face"]
# sc = ax.scatter(cps_face[:, 0], cps_face[:, 1], cps_face[:, 2], c=cp_forces_face, cmap='viridis')

# cbar = fig.colorbar(sc, ax=ax, pad=0.1)  # pad adjusts spacing
# cbar.set_label('Force')

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('Contact Points')
# ax.legend()

# plt.show()