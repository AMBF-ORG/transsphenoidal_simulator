import rosbag
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Patch
import matplotlib.image as mpimg
from matplotlib.ticker import MaxNLocator
import yaml


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

def getRPY(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # uses extrinsic r, p, y or intrinsic y, p , r
    cR, cP, cY = np.cos(roll), np.cos(pitch), np.cos(yaw)
    sR, sP, sY = np.sin(roll), np.sin(pitch), np.sin(yaw)

    m = np.zeros((3, 3))
    m[0, 0] = cY * cP
    m[0, 1] = cY * sP * sR - sY * cR
    m[0, 2] = cY * sP * cR + sY * sR

    m[1, 0] = sY * cP
    m[1, 1] = sY * sP * sR + cY * cR
    m[1, 2] = sY * sP * cR - cY * sR

    m[2, 0] = -sP
    m[2, 1] = cP * sR
    m[2, 2] = cP * cR

    return m

def getTF(rot: np.ndarray, pos: np.ndarray) -> np.ndarray:
    m = np.eye(4)

    m[:3, :3] = rot
    m[:3, 3] = pos

    return m

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

def getCameraRot(pos: np.ndarray, lookAt: np.ndarray, up: np.ndarray) -> np.ndarray:

    backward = (pos - lookAt)
    backward /= np.linalg.norm(backward)

    up /= np.linalg.norm(up)

    right = np.cross(up, backward)
    right /= np.linalg.norm(right)

    up = np.cross(backward, right)

    return np.column_stack((backward, right, up))


# bag_file = Path('~/Documents/recordings/userstudy/TS_data_user01_2025-10-28-09-15-55.bag').expanduser()
# USER="user01"
# bag_file = Path('~/Documents/recordings/userstudy/TS_data_user02_2_2025-10-28-11-36-06.bag').expanduser()
# USER="user02"
bag_file = Path('~/Documents/recordings/userstudy/TS_data_user03_rightnose_2025-10-28-15-27-18.bag').expanduser()
USER="user03"

T_w_i = None
img_dims = None
with open(Path("~/transsphenoidal_simulator/volume/volume.yaml").expanduser(), "r") as f:
    data = yaml.safe_load(f)
    volname = data["volumes"][0]
    dims = data[volname]["dimensions"]
    voldims = np.array((dims["x"], dims["y"], dims["z"]))
    img_count = data[volname]["images"]["count"]
    img = mpimg.imread(Path("~/transsphenoidal_simulator/volume").expanduser() / data[volname]["images"]["path"] / "slice00.png")
    img_height, img_width = img.shape[:2]

    img_dims = np.array([img_height, img_width, img_count])

    imgscale = np.zeros((3, 3))
    # ambf x axis is image y axis
    imgscale[0, 0] = dims["x"] / img_height
    imgscale[1, 1] = dims["y"] / img_width
    imgscale[2, 2] = dims["z"] / img_count

    imgpos = -voldims / 2

    T_v_i = getTF(imgscale, imgpos)

    pos = data[volname]["location"]["position"]
    volpos = np.array((pos["x"], pos["y"], pos["z"]))
    rot = data[volname]["location"]["orientation"]
    volrot = getRPY(rot["r"], rot["p"], rot["y"])

    T_b_v = getTF(volrot, volpos)

    bodyname = data["bodies"][0]
    pos = data[bodyname]["location"]["position"]
    bodypos = np.array((pos["x"], pos["y"], pos["z"]))
    rot = data[bodyname]["location"]["orientation"]
    bodyrot = getRPY(rot["r"], rot["p"], rot["y"])

    T_w_b = getTF(bodyrot, bodypos)

    T_w_i = T_w_b @ T_b_v @ T_v_i

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

T_c_i = T_c_w @ T_w_i

indices_ls = []
colors_ls = []
topic_name = '/ambf/env/plugin/volumetric_drilling/drill/voxels_removed'
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        for index in msg.indices:
            indices_ls.append((index.x, index.y, index.z))
        for color in msg.colors:
            colors_ls.append((color.r, color.g, color.b, color.a))

indices = np.column_stack(indices_ls)
colors = np.column_stack(colors_ls)

colors_unq = np.unique(colors, axis=1)
sphenoid_mask = (colors == colors_unq[:, [3]]).all(axis=0)
sphenoid_col = np.array([[255, 255, 167, 255]]).T / 255

colors[:, sphenoid_mask] = sphenoid_col

indices_rel = indices - indices.min(axis=1, keepdims=True)
xmax, ymax, zmax = indices_rel.max(axis=1) + 1

filled = np.zeros((xmax, ymax, zmax), dtype=bool)
facecolors = np.zeros((xmax, ymax, zmax, 4), dtype=float)

x, y, z = indices_rel
filled[x, y, z] = True
facecolors[x, y, z] = colors.T

indices_hg = np.vstack((indices, np.ones((1, indices.shape[1]))))

voxels_hg = make_homogenous(np.tile(np.arange(0, max(filled.shape) + 1), (3, 1)) + indices.min(axis=1, keepdims=True))
voxels = T_c_i @ voxels_hg


# manually swap around rows to match rotation of voxel vertex points
# means that angles which are not 90 deg multiples won't work
filled_rot = filled.transpose(1, 0, 2).transpose(2, 1, 0)
facecolors_rot = facecolors.transpose(1, 0, 2, 3).transpose(2, 1, 0, 3)
x = voxels[0, :(filled.shape[2] + 1)]
y = voxels[1, :(filled.shape[0] + 1)]
z = voxels[2, :(filled.shape[1] + 1)]

X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(16, 9))
ax.voxels(X, Y, Z, filled_rot, facecolors=facecolors_rot)

ax.set_xlabel('X (m)', labelpad=35)
ax.set_ylabel('Y (m)', labelpad=35)
ax.set_zlabel('Z (m)', labelpad=45)
ax.set_title('Voxels Removed')
ax.set_box_aspect([1,1,1])
setAxesScaleEqual(ax)

labels = ['Nasal Tissue', 'Pituitary Gland', 'Target Bony Tissue', 'Sphenoid Bone']

legend_elements = [
    Patch(facecolor=color, edgecolor='black', label=label)
    for color, label in zip(np.unique(colors, axis=1).T, labels)
]


ax.view_init(elev=30, azim=-45)


ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
ax.set_xticklabels([f"{x:.3f}" for x in ax.get_xticks()], rotation=0, ha='right')
ax.set_yticklabels([f"{x:.3f}" for x in ax.get_yticks()], rotation=-0, ha='left')
ax.set_zticklabels([f"{x:.3f}" for x in ax.get_zticks()], rotation=0, ha='left')

plt.savefig(USER + "_voxels.pdf")

plt.show()

fig_legend, ax = plt.subplots(figsize=(4,1))
ax.axis('off')  # turns off axes, ticks, frame
plt.legend(handles=legend_elements, ncol=5)
fig_legend.savefig("voxels_legend.pdf", bbox_inches='tight')
plt.close(fig_legend)