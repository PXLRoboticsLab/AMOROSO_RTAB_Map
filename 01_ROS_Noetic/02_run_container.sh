#!/bin/bash

CURRENT_IP=$(hostname -I | awk '{print $1}')

if ! command -v glxinfo &> /dev/null
then
    echo "glxinfo command  not found! Execute \'sudo apt install mesa-utils\' to install it."
    exit
fi

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`


if [ $vendor == "NVIDIA" ]; then
    docker run -it --rm \
        --name ros_noetic_rtab \
        --device /dev/snd \
        --network host \
        --env="DISPLAY" \
        -v `pwd`/../Commands/bin:/home/user/bin \
        -v `pwd`/../Data:/home/user/Data  \
        --volume=/dev:/dev \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -env="XAUTHORITY=$XAUTH" \
        --gpus all \
        -e ROS_MASTER_URI=http://$CURRENT_IP:11311 \
        -e ROS_IP=$CURRENT_IP \
        ros_noetic_rtab:latest  \
        bash
else
    docker run --privileged -it --rm \
        --name ros_noetic_rtab \
        --network host \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        -v `pwd`/../Commands/bin:/home/user/bin \
        -v `pwd`/../Data:/home/user/Data  \
        --volume=/dev:/dev \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        -e ROS_MASTER_URI=http://$CURRENT_IP:11311 \
        -e ROS_IP=$CURRENT_IP \
        ros_noetic_rtab:latest  \
        bash
fi
