#!/bin/bash

# Get the current IP address
CURRENT_IP=$(hostname -I | awk '{print $1}')

# Update the .env file for ROS_MASTER_URI and ROS_IP
sed -i "s/\(ROS_MASTER_URI=http:\/\/\)[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+/\1$CURRENT_IP/" .env
sed -i "s/\(ROS_IP=\)[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+/\1$CURRENT_IP/" .env

echo "Updated .env file with current IP: $CURRENT_IP"


(cd ./01_ROS_Noetic; ./02_run_container.sh)
