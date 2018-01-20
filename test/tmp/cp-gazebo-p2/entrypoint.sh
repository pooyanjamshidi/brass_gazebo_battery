#!/bin/bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source devel/setup.bash

sudo ln -sf /dev/null /dev/raw1394

# fix: https://github.com/leonid-shevtsov/headless/issues/47
mkdir -p /tmp/.X11-unix
sudo chmod 1777 /tmp/.X11-unix
sudo chown root /tmp/.X11-unix

# launch a headless X-server
Xvfb :1 -screen 0 1600x1200x16 &
export DISPLAY=:1.0

exec "$@"
