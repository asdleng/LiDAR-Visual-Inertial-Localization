# VIOLM
VIOLM is a Visual-Inertial Odometry (VIO) system designed to running in a LiDAR-generated point cloud map. The features of the system are as follows:
- Rather than integrating a registration module into an existing VIO framework, our approach directly projects the map points into the camera.
- The direct method and feature-based method are jointly utilized for system accuracy and robustness.
- A 2D-3D line corresponding module and a 3D point alignment module are designed to further improve the pose estimation.

![Image text](https://github.com/asdleng/VIOLM/blob/main/img/v1_01.png)

## Prerequisites
- ROS
- Eigen
- Ceres
- Sophus
- rpg_vikit(https://github.com/uzh-rpg/rpg_vikit)
- livox_ros_driver(https://github.com/Livox-SDK/livox_ros_driver)
- Optional: online photometric calibration(https://github.com/tum-vision/online_photometric_calibration)
Please use the online photometric calibration above to generate corrected images in case of running the EuRoC dataset.

## Build
```bash
mkdir your_workspace_name
cd your_workspace_name
mkdir src
cd src
```
First, you should build the livox_ros_driver and rpg_vikit
```bash
git clone https://github.com/uzh-rpg/rpg_vikit.git
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin_make
```
If error occured, you should use -DCATKIN_WHITE_LIST to specify the build order.
Now you can clone this repository and build.
```bash
cd src
git clone git@github.com:asdleng/VIOLM.git
cd ..
catkin_make
```

## Run
```bash
cd your_workspace_name
source devel/setup.bash
roslaunch vio_in_lidar_map xxxx.launch
```

## Rosbag Example
- EuRoC dataset(https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
The V1 room of EuRoC dataset contains a laser-scan-built point cloud map. So V101~V203 is suitable for our system.




