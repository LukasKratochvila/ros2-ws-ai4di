#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/ros/ros2-ws-ai4di/install/setup.bash

cd /home/ros/ros2-ws-ai4di/
echo "Launching rpi-cam"
ros2 launch launch/Launch_rpi-cam.py
