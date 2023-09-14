#!/bin/bash


source /opt/ros/noetic/setup.bash
roslaunch rtabmap_launch rtabmap.launch visual_odometry:=false \
                                        icp_odometry:=false \
                                        stereo:=false \
                                        depth:=false \
                                        subscribe_scan:=true \
                                        rgbd_sync:=false \
                                        stereo_rectify_active:=false \
                                        scan_topic:=/scan \
                                        frame_id:=base_link \
                                        odom_frame_id:=odom \
                                        map_frame_id:=map \
                                        debug:=true