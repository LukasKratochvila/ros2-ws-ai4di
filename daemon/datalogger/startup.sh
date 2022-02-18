#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/ros/ros2-ws-ai4di/install/setup.bash

cd /home/ros/ros2-ws-ai4di/
n=0
createDate=`date +"%Y-%m-%d_%H:%M"`
filename="/home/ros/ros2-ws-ai4di/${createDate}"

while [[ -d $filename ]]
 do
  echo "Used: ${createDate}"
  echo "Tries: ${createDate}_${n}"
  filename="${createDate}_${n}"
  ((n=n+1))
 done 

echo "Data dir: $filename"
mkdir "$filename"
cd "$filename"
mkdir image
mkdir pcl

echo "setting up PTP"
ptpd -i eth0 -M #& --foreground

sleep 12
echo "Launching measurement"
ros2 launch ../Launch_MTDL.py
