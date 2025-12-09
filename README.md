# Transsphenoidal Surgical Simulator
This is the official repository for TRIDENT: Transsphenoidal surgical simulation for Realistic Interactive Drilling and Endoscope Navigation Training. 

[Paper](link_to_be_added) | [Video](link_to_be_added)

```bibtex
@article{ishida2025TRIDENT,
    author = {Ishida*, Hisashi and Ying*, Andrew and Ding, Andy S. and Heo, Yub and Wang,   
    Jonathan and Vedula, Swaroop and Ishida, Wataru and Taylor, Russell and Ishii, Masaru 
    and Munawar, Adnan},
    title = {TRIDENT: Transsphenoidal Surgical Simulator for Realistic Interactive Drilling 
    and Endoscope Navigation Training.},
    note = {Under Review},
    year = 2025
}
```
## Overview

This repository presents TRIDENT, an open-source virtual simulation system developed for training and evaluating transsphenoidal surgery. The simulator is designed to replicate the complex anatomical and procedural challenges of transsphenoidal surgery. The system also features dual-instrument control of a surgical drill and endoscope within a narrow anatomical corridor, simulating the clinical constraints of this procedure. The system incorporates three types of anatomical models: rigid structures, deformable soft tissues, and drillable volumes for bone drilling. High-fidelity visual rendering and haptic feedback enhance realism and support accurate tool-tissue interaction.

![image](media/MainFigure_new.png)

The simulator builds upon the FIVRS bone drilling simulator (https://github.com/LCSR-SICKKIDS/volumetric_drilling), which itself is a plugin built on top of Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al.


## 1. Installation Instructions:
### 1.1 Install and Source AMBF 2.0

Clone and build `ambf-2.0` branch.
```bash
git clone git@github.com:hisashiishida/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```

**⚠️ Important:** Be sure to clone the fork, which has some important changes for use with the dVRK MTMs.

Note that depth and image recording are enabled by default (in camera ADFs) and these features only work on Linux with ROS installed. Additionally, the following packages must be installed prior to building to AMBF:

```bash
cv-bridge # Can be installed via apt install ros-<version>-cv-bridge
image-transport # Can be installed via apt install ros-<version>-image-transport
```

Build and source ambf (make sure you're on branch ambf-2.0 before building) as per the instructions on AMBFs wiki: https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.

### 1.2 Clone and Build Simulator
``` bash
git clone git@github.com:AMBF-ORG/transsphenoidal_simulator.git
cd transsphenoidal_simulator
mkdir build
cd build
cmake ..
make
```

### 1.3 Clone and Build Camera Distortion Plugin
Follow the instructions here: https://github.com/AMBF-ORG/ambf_camera_distortion_plugin

Note that `transsphenoidal_simulator/plugin_config/ambf_camera_distortion_plugin/` and `transsphenoidal_simulator/ADF/endoscope_camera.yaml` reference this plugin.

**⚠️ Important:** We also need to create a symlink to the plugin:
``` bash
cd ~/volumetric_drilling/plugin
ln -s ~/ambf_camera_distortion_plugin
```
Note this symlink has a habit of breaking after Git operations and may need to be relinked.

## 2 Running the Plugin with ambf_simulator:
The transsphenoidal simulator is a plugin that is launched on top of the AMBF simulator along with other AMBF bodies, described by AMBF Description Format files (ADFs), as will be demonstrated below. The `libvolumetric_drilling.so` plugin is initialized in the `launch.yaml` file and can be commented out for the purpose of debugging the ADF files.   

Below are instructions as to how to load different volume and camera options. The -l tag used below allows user to run indexed multibodies that can also be found in the `launch.yaml` under the `multibody configs:` data block. More info on launching the simulator can be found in the AMBF Wiki:  

https://github.com/WPI-AIM/ambf/wiki/Launching-the-Simulator  
https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots  
https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments  

Note that the executable binary,`ambf_simulator`, is located in `ambf/bin/lin-x86_64`.

### 2.1 Bash Script
The bash script `bash/transsphenoidal.bash` can be used to easily run the simulator.

### 2.2 Manipulating Endoscope and Drill

Control of the endoscope with the haptic device may be toggled using the `<space>` key however this causes the orientation in the simulator to snap the haptic device orientation.

### 2.3 Changing Scene Parameters
All the relevant ADF scene objects are in the ADF folder and can be modified as needed. For example, camera intrinsics can be adjusted via the field view angle and image resolution parameters of Camera ADFs.

#### 2.4.1 Keyboard Navigation

| # | Linear Motion of Tool | Description                                  |
|---|-----------------------|----------------------------------------------|
| 1 | [Ctrl+W]              | Moves vertically upward w.r.t. camera        |
| 2 | [Ctrl+S]              | Moves vertically downward w.r.t. camera      |
| 3 | [Ctrl+A]              | Moves horizontally left w.r.t. camera        |
| 4 | [Ctrl+D]              | Moves horizontally right w.r.t. camera       |
| 5 | [Ctrl+I]              | Moves in the forward direction w.r.t camera  |
| 6 | [Ctrl+K]              | Moves in the backward direction w.r.t camera |


| # | Angular Motion of Tool | Description                                     |
|---|------------------------|-------------------------------------------------|
| 1 | [Num 8]                | Rotates towards upward direction w.r.t tool     |
| 2 | [Num 5]                | Rotates towards downward direction w.r.t. tool  |
| 3 | [Num 4]                | Rotates towards the left direction w.r.t. tool  |
| 4 | [Num 6]                | Rotates towards the right direction w.r.t. tool |


| # | Miscellaneous | Description                                                                        |
|---|---------------|------------------------------------------------------------------------------------|
| 1 | [Ctrl+\<Space\>]      | Toggle control mode between drill and endoscope when there is < 2 haptic devices       |
| 2 | [Ctrl+F]      | Free the MTMs, allowing haptic feedback       |
| 3 | [Ctrl+H]      | Hold the MTMs, disallowing haptic feedback       |

#### 2.4.2 Navigation using SpaceNav Plugin
Follow the instructions here: https://github.com/LCSR-CIIS/ambf_spacenav_plugin.

`volumetric_drilling/plugin_config/ambf_spacenav_plugin/` defines the config used by the plugin and can be used to speed up the rotation or translation.

#### 2.4.3 Navigation using dVRK MTMs
Follow the instructions here: https://dvrk.readthedocs.io/main/pages/software/compilation/ros1.html#catkin-workspace-clone-and-build.

**⚠️ Important:** We also need to copy the system config to the workspace, since it references paths in the directory, i.e. `cp plugin_config/TS_system-MTML-MTMR.json ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-dVRK`

#### 2.4.4 Navigation using Phantom Omni / Geomagic Touch

Phantom Omni drivers can be installed from here: https://github.com/jhu-cisst-external/phantom-omni-1394-drivers.

See here for instructions to set up the device: https://github.com/LCSR-CIIS/AMBF_helper/blob/main/Hardware_installation.md.

In the case of the Phantom Omni, be sure to change the "PHANToM Model" to "Omni" when running `sudo PHANToMConfiguration`.

#### 2.5.1 Contact Force Low Pass Filter Parameters
The frequency cutoff used to smooth the contact force and torque for the drill and endoscope, `FC_force` and `FC_torque`, can be set in the `drill_manager.cpp` and `endo_manager.cpp`.

They are currently set for the dVRK MTMs and may need to be tuned if the Geomagic Touch is used instead.

#### 2.5.2 Force Scaling Parameters
The gains (`volumeBasedForceConst`, `rigidbodyBasedForceConst`, `rigidbodyBasedTorqueConst`) and maximum magnitudes (`maxForceConst`, `maxTorqueConst`) for the force and torque can be set in `volumetric_drilling.cpp`.

They are currently set for the dVRK MTMs and may need to be tuned if the Geomagic Touch is used instead.

#### 2.5.3 PD Parameters
Linear and angular gains for the proportional and derivative components of the PD controller can be set in `drills.yaml` and `endoscope_camera.yaml` under `4mm` and `Endoscope 35 degree in REMS`.

#### 2.5.4 Soft body Parameters
Damping, friction, and pose matching coefficients (`kDP`, `kDF`, `kMT`) for the soft body can be set in nose.yaml.

#### 2.5.2 Force Scaling Parameters
The gains (`volumeBasedForceConst`, `rigidbodyBasedForceConst`, `rigidbodyBasedTorqueConst`) and maximum magnitudes (`maxForceConst`, `maxTorqueConst`) for the force and torque can be set in `volumetric_drilling.cpp`.

They are currently set for the dVRK MTMs and may need to be tuned if the Geomagic Touch is used instead.

#### 2.6.1 Default Volume
For convenience, a default volume taken from a CT scan of a surgical training phantom has been provided. This does not contain all the structures segmented for the private patient volume described in the paper.

| # | Structure | Color | Type |
|---|---------------|-----------------------------------------|-----|
| 1 | Face  | Brown | Rigid |
| 2 | Nose     | Brown | Soft |
| 3 | Nasal Tissue      | Brown | Volume |
| 4 | Pituitary Gland      | Pink | Volume |
| 5 | Carotid Arteries      | Red | Volume |
| 6 | Optic Nerves | Yellow | Volume |

#### 2.6.2 Adding Volumes
Additional volumes can be created by modifying the launch file and adding and option for the new volume directory. https://github.com/AMBF-ORG/nrrd_to_adf may be used to facilitate exporting segmentations from 3D Slicer to AMBF.
