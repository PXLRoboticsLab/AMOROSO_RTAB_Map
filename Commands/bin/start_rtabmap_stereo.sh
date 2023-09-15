#!/bin/bash

source /opt/ros/noetic/setup.bash
roslaunch rtabmap_launch rtabmap.launch rtabmap_args:="--Odom/MinInliers 1" \
                                        stereo:=false \
                                        depth:=true \
                                        visual_odometry:=false \
                                        icp_odometry:=false \
                                        rgb_topic:=/camera/rgb/image_raw \
                                        depth_topic:=/camera/depth/image_raw \
                                        camera_info_topic:=/camera/rgb/camera_info \
                                        imu_topic:=/imu \
                                        odom_topic:=/odom \
                                        frame_id:=base_link