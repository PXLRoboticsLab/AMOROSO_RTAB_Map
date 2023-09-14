#!/bin/bash

# Get the current IP address
CURRENT_IP=$(hostname -I | awk '{print $1}')

# Update the .env file
sed -i "s/<Your-Computer-IP>/$CURRENT_IP/g" .env

echo "Updated .env file with current IP: $CURRENT_IP"


(cd ./01_ROS_Noetic; ./02_run_container.sh)
