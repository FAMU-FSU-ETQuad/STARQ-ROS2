#!/bin/bash

# TODO

# Set up user
# - usermod -aG dialout pi

# install dependencies (install_deps.bash)
# - python3
# - pip
# - odrive (pip)
# - can / cantools (pip)
# - serial

# install ros2 humble (install_ros2_humble.bash)
# - colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# - ROS_DOMAIN_ID=2
# - Multicast/UDP settings?

# set up can (install_can.sh)
# - install can-utils
# - '/boot/firmware/config.txt'
# -- dtparam=spi=on
# -- dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
# -- dtoverlay=spi0-hw-cs
# - *Needs reboot*

# set up startup script
# - mv startup.service to /etc/systemd/system/startup.service
# - Run on boot:
# -- set can0 up type can bitrate 500000
# -- git pull origin main
# -- colcon build
# -- source install/setup.bash
# -- ros2 launch