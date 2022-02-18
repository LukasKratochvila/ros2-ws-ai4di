#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/ros/ros2-ws-ai4di/install/setup.bash

echo "Setting up PTP"
ptpd -i eth0 -M #& --foreground

sleep 12
cd /home/ros/ros2-ws-ai4di/
echo "Launching rpi-lidar"
ros2 launch launch/Launch_rpi-lidar.py
