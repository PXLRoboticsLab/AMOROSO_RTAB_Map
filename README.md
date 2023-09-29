# RTAB-Map Docker Container

This repository contains Dockerfiles and scripts to run RTAB-Map using ROS Noetic within a Docker container.


## Getting Started

### Build Docker Images

1. Run `001_build_images.sh` to build the initial Docker images.

### Rebuild Docker Images

1. If needed, use `002_rebuild_images.sh` to rebuild the images.

### Start RTAB-Map with ROS Noetic

1. Run `003_start_ros_noetic_rtab.sh` to start the RTAB-Map service within a ROS Noetic environment.

### Attach Bash Session to Running Container

1. Use `005_attach_bash_to_ros_noetic_rtab.sh` to attach a bash session to a running container.

## Advanced Usage

### Commands

- `start_rtabmap_lidar_only.sh`: Starts RTAB-Map with LiDAR.
- `start_rtabmap_rgbd.sh`: Starts RTAB-Map with stereo vision.

### Volumes

- Use the `Volumes` directory to mount external storage.

## Data

- Place any required data in the `Data` directory.

## License

MIT
