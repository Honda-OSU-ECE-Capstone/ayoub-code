#!/usr/bin/env bash
set -eu
cd "${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ -f .venv/bin/activate ]; then source .venv/bin/activate; fi
unset HEADLESS
unset PX4_GZ_STANDALONE
export GZ_IP=127.0.0.1
export PX4_GZ_MODEL_POSE="0,0"
exec make px4_sitl gz_x500_mono_cam_down
