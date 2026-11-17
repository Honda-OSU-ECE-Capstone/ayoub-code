#!/usr/bin/env bash
set -eu
cd "${PX4_DIR:-$HOME/PX4-Autopilot}"
# Six meters west, with its own downward camera and MAVLink instance.
exec env GZ_IP=127.0.0.1 PX4_GZ_WORLD=default PX4_GZ_STANDALONE=1 \
    PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="-6,0" \
    PX4_SIM_MODEL=gz_x500_mono_cam_down \
    ./build/px4_sitl_default/bin/px4 -i 1
