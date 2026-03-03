#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.." || exit 1

ambf_simulator --launch_file launch.yaml -l 0,1,2,3,4 --mute 1 --fp /dev/js5 --nt 1 -t 1 -p 240
