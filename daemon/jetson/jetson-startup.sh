#!/bin/bash

#cd /home/ros/jetson-containers

#echo "In next terminal run: sudo docker cp con_backup/ros-ws/ dolar(sudo docker ps -lq):/"
#echo "In docker run: cd ros-ws && source install/setup.bash && ros2 launch launcher.py"
#echo
#./scripts/docker_run.sh -c dustynv/ros:foxy-ros-base-l4t-r32.6.1

#echo "In docker run: cd ros-ws && source install/setup.bash && ros2 launch launcher.py"
#echo

CONTAINER_IMAGE=""
USER_VOLUME=""
USER_COMMAND=""

# give docker root user X11 permissions
#sudo xhost +si:localuser:root

# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
#XAUTH=/tmp/.docker.xauth
#xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
#chmod 777 $XAUTH

# run the container
docker run -d --runtime nvidia -t --rm --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix ros:foxy
    #-v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
    

sleep 2
CONTAINER_ID=$(sudo docker ps -lq)
docker exec -t -w "/ros-ws" $CONTAINER_ID bash -c "source install/setup.bash && ros2 launch launcher.py"
#sudo docker attach $CONTAINER_ID
