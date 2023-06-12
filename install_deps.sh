#!/bin/bash

apt-get update
apt-get install -y python3-pip can-utils
pip3 install odrive can cantools serial