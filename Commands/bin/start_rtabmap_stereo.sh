#!/bin/bash

source /opt/ros/noetic/setup.bash
roslaunch rtabmap_launch rtabmap.launch stereo:=false \
                                        depth:=true \
                                        rgb_topic:=/camera/color/image_raw \
                                        depth_topic:=/camera/depth/image_raw \
                                        camera_info_topic:=/camera/color/camera_info \
                                        imu_topic:=/imu \
                                        odom_topic:=/odom \
                                        frame_id:=camera_link