# RTAB-Map Docker Container

This repository contains Dockerfiles and scripts to run RTAB-Map using ROS Noetic within a Docker container.


## Getting Started

### Build Docker Images

Run `001_build_images.sh` to build the initial Docker images.
```
./001_build_images.sh
```

### Rebuild Docker Images

If needed, use `002_rebuild_images.sh` to rebuild the images.
```
./002_rebuild_images.sh
```

### Start RTAB-Map container with ROS Noetic

Run `003_start_ros_noetic_rtab.sh` to start the RTAB-Map container within a ROS Noetic environment.
```
./003_start_ros_noetic_rtab.sh
```

### Attach Bash Session to Running Container

Use `005_attach_bash_to_ros_noetic_rtab.sh` to attach a bash session to a running container.
```
./005_attach_bash_to_ros_noetic_rtab.sh
```

### Build the workspace

Once inside the container, go to the `unitree_ws` directory, run `catkin_make` to build the workspace and source the `setup.bash` file to apply the changes made by catkin_make to your terminal.
```
cd ~/unitree_ws
catkin_make
source devel/setup.bash
```

## Launch the robot

Launch the robot in the ROSNoeticDocker container **with host network**
- Terminal 1:
    ```
    export TURTLEBOT3_MODEL=waffle
    start_turtlebot3_gazebo_world.sh
    ```
- Terminal 2:
    ```
    start_turtlebot3_gazebo_rviz.sh
    ```
- Terminal 3:
    ```
    start_turtlebot3_teleop_key.sh
    ```
> ⚠️ Pro tip: if you want to be lazy, save some time by creating a launch file to launch all the above scripts in a single command. 😴


## RTAB-Map SLAM

#### LiDAR:
```
start_rtabmap_lidar_only.sh
```
|Parameter|Info|
|---|---|
|`scan_topic:=/scan`|Requires the topic on which the LiDAR data gets published|
|`frame_id:=base_link`|Requires the frame that serves as the base reference of the robot|
|`odom_frame_id:=odom`|Requires the frame that serves as the odometry reference of the robot|
|`map_frame_id:=map`|Requires the frame that serves as the map reference|


#### Camera: 
```
start_rtabmap_rgbd.sh
```
|Parameter|Info|
|---|---|
|`rgb_topic:=/camera/rgb/image_raw`|Requires the topic on which the raw RGB image is published|
|`depth_topic:=/camera/depth/image_raw`|Requires the topic on which the depth image is published|
|`camera_info_topic:=/camera/rgb/camera_info`|Requires the topic on which the camera info is published|
|`imu_topic:=/imu`|Requires the topic on which the IMU data of the robot is published|
|`odom_topic:=/odom`|Requires the topic on which the odometry data of the robot is published|
|`frame_id:=base_link`|Requires the frame that serves as the base reference of the robot|

> ⚠️ Pro tip: Read the source code of the scripts to understand what they do, what other launch scripts from other packages they invoke, what arguments they take, and how they work. This knowledge will help you debug any future issues you ~~may~~ will encounter 🤓. If you need more information on RTAB-Map, check out the [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros).



## Development

### Debugging

There are a lot of things that can go wrong, like, **A LOT** 🙃. Here is a list of things to try ~~if~~ when you run into any common issues such as incorrectly configured topics or unconnected tf trees.

- If you get an error like `ERROR: Unable to communicate with master!`, try running `source /opt/ros/noetic/setup.bash` and try again.
    ```
    source /opt/ros/noetic/setup.bash
    ```
- If you get an error like `ERROR: Unable to communicate with master!`, try running `source ~/unitree_ws/devel/setup.bash` and try again.
    ```
    source ~/unitree_ws/devel/setup.bash
    ```
- inspect topics with `rostopic list`
    ```
    rostopic list
    ```
- inspect messages with `rostopic echo /topic_name`
    ```
    rostopic echo /topic_name
    ```
- inspect nodes with `rosnode list`
    ```
    rosnode list
    ```
- inspect node info with `rosnode info /node_name`
    ```
    rosnode info /node_name
    ```
- inspect the rqgraph with `rosrun rqt_graph rqt_graph`
    ```
    rosrun rqt_graph rqt_graph
    ```
- inspect the TF tree with `rosrun rqt_tf_tree rqt_tf_tree`
    ```
    rosrun rqt_tf_tree rqt_tf_tree
    ```
- inspect the TF tree with `rosrun tf view_frames`
    ```
    rosrun tf view_frames
    ```
- inspect the TF tree with `evince frames.pdf`
    ```
    evince frames.pdf
    ```
- inspect the TF tree with `rosrun tf tf_monitor`
    ```
    rosrun tf tf_monitor
    ```
- inspect the TF tree with `rosrun tf tf_echo /frame1 /frame2`
    ```
    rosrun tf tf_echo /frame1 /frame2
    ```
- search for a package with `roscd package_name`
    ```
    roscd package_name
    ```


### 6 golden rules for life you must recite every day before bed so you never forget  

> **`Have you tried turning it off and on again?`** 🧐 

> **Logs and error messages exist for a reason.** 🤨

> **If you are stuck, Google is your good ol' friend.** 😏 

> **If you are still stuck, ChatGPT is your new bestie.** 😍

> **If it's on the internet, it must be true! ~ Albert Einstein** 🤔

>  **In case of fire:**
```
git add *
git commit -m '🔥'
git push
```

## Volumes

- Use the `Volumes` directory to mount **external storage**.
- If you are using a **remote server**, you can use the `Volumes` directory to **share files** between your local machine and the remote server.
- If you clone a repository to your **local machine**, you can use the `Volumes` directory to **share files** between your local machine and the Docker container.
- Beware of commiting a repository within a repository! Make sure to create a submodule or remove the `.git` directory or add the `Volumes` directory to the `.gitignore` file before commiting.


## Data

- Place any required data in the `Data` directory.

## License

MIT
