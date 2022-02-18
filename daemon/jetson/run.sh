#!/usr/bin/env bash

cd jetson-containers

#echo "In next terminal run: sudo docker cp con_backup/ros-ws/ dolar(sudo docker ps -lq):/"
#echo "In docker run: cd ros-ws && source install/setup.bash && ros2 launch launcher.py"
#echo
#./scripts/docker_run.sh -c dustynv/ros:foxy-ros-base-l4t-r32.6.1

echo "In docker run: cd ros-ws && source install/setup.bash && ros2 launch launcher.py"
echo
./scripts/docker_run.sh -c ros:foxy


