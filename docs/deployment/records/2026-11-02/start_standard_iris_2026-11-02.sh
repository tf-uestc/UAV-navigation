#!/usr/bin/env bash
set -e
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH
export DISPLAY=:1 XAUTHORITY=/root/.Xauthority
export GAZEBO_MODEL_DATABASE_URI=''
export NO_PXH=1 HEADLESS=1 VERBOSE_SIM=1
cd /root/autodl-tmp/uav/PX4-Autopilot
exec /opt/VirtualGL/bin/vglrun -d egl0 bash Tools/simulation/gazebo-classic/sitl_run.sh "$PWD/build/px4_sitl_default/bin/px4" none iris empty "$PWD" "$PWD/build/px4_sitl_default"
