#!/bin/bash
# -t 1 -p 60 
# use fp to avoid the simulator thinking that there is a footpedal
ambf_simulator --launch_file launch.yaml -l 0,1,2,3,4 --mute 1 --fp /dev/js5 --nt 1 -t 1 -p 240 --tf_list src/transsphenoidal_simulator/plugin_config/ambf_tf_plugin/tf_ros.yaml
# --plugin ../ambf_spacenav_plugin/build/libspacenav_plugin.so --spf plugin_config/ambf_spacenav_plugin/config.yaml



