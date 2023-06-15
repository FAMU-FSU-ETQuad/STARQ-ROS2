# CISCOR Lab Boom

## Setup

### Raspberry Pi ###
A. Flash Raspberry Pi with [Ubuntu 22.04](https://ubuntu.com/raspberry-pi)

Username: `pi`\
Password: *none*\
Wifi: `Vicon`

B. Download project from git

```
sudo apt update && sudo apt upgrade -y
sudo apt install git
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/FAMU-FSU-ETQuad/STARQ-ROS2.git boom_packages
```

C. Run installation script

```
cd ~/ros2_ws/src/boom_packages/boom_rpi
sudo ./boom_install.bash
sudo reboot
```

### Teensy ###

- ?From Windows or install Teensy boot loader on Rpi?

### Electrical Wiring ##

- !Put in picture
- Last ODrive controller in CAN bus has switches up

### Configuring Motors ###

- ?From user device or from Rpi?

### Setting up station (MATLAB) ###

- Install MATLAB
- Install ROS Toolbox
- Install BoomController MATLAB class
- Add to path