#!/bin/bash
sudo ip link set can0 up type can bitrate 500000
cd /home/pi/ros2_ws/src/boom_packages && git pull origin main
source /opt/ros/humble/setup.bash
cd /home/pi/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source /home/pi/ros2_ws/install/setup.bash
ros2 launch /home/pi/ros2_ws/src/launch/boom.launch
