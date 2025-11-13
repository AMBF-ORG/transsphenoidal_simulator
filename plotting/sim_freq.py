import rosbag
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

bag_file = Path('~/Documents/recordings/procedure_3.bag').expanduser()
topic_name = '/ambf/env/World/State'

data = {}
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
            data.setdefault("graphics_loop_freq", []).append(msg.graphics_loop_freq)
            data.setdefault("dynamic_loop_freq", []).append(msg.dynamic_loop_freq)
            data.setdefault("sim_step", []).append(msg.sim_step)
            data.setdefault("wall_time", []).append(msg.wall_time)
            data.setdefault("sim_time", []).append(msg.sim_time)

# List of keys to plot against sim_time
keys_to_plot = ["graphics_loop_freq", "dynamic_loop_freq", "sim_step", "wall_time"]

# Create subplots: 1 row per key
fig, axs = plt.subplots(len(keys_to_plot), 1, figsize=(10, 8), sharex=True)
# fig.suptitle("Sim Time vs Various Metrics")

sim_time = data["sim_time"]

# Loop through each key and corresponding subplot axis
for i, key in enumerate(keys_to_plot):
    axs[i].plot(sim_time, data[key])
    axs[i].set_ylabel(key.replace('_', ' '))
    axs[i].grid(True)

# Label x-axis on the bottom subplot only
axs[-1].set_xlabel("sim_time")

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to fit title
plt.show()