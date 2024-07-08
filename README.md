# LiDAR-Visual-IMU Fusion Localization
<img src="https://github.com/asdleng/LiDAR-Visual-Inertial-Localization/blob/master/img/systemoverview.jpg" width="50%">


This project presents a robust LiDAR-Visual-Inertial localization system running within a LiDAR-generated point cloud map, named LM-LVIL. The code is developed based on FAST-LIVO and FAST-LIO2, under the framework of IESKF. Photometric measurements, projection measurements, line measurements, and 3D point alignment measurements are considered to enhance the system's accuracy and robustness.


## Prerequisites
- ROS
- Eigen
- Ceres
- Sophus
- rpg_vikit(https://github.com/uzh-rpg/rpg_vikit)
- livox_ros_driver(https://github.com/Livox-SDK/livox_ros_driver)

For Ubuntu 22.04 users, it is recommended to use Docker.

## Build
```bash
mkdir your_workspace_name 
```
First, you should install Sophus
```bash
cd your_workspace_name && cd src
git clone https://github.com/strasdat/Sophus.git
cd Sophus && mkdir build && cd build
cmake .. && make -j8
```
Then you should build the livox_ros_driver and rpg_vikit
```bash
cd your_workspace_name && cd src
git clone https://github.com/uzh-rpg/rpg_vikit.git
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone git@github.com:asdleng/LiDAR-Visual-Inertial-Localization.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver"
catkin_make -DCATKIN_WHITELIST_PACKAGES="vikit_common"
catkin_make -DCATKIN_WHITELIST_PACKAGES="vikit_ros"
catkin_make -DCATKIN_WHITELIST_PACKAGES="lmlvil"
```
If error occured, you should use -DCATKIN_WHITELIST_PACKAGES to specify the build order.


## Map Build
You should build the point cloud map using LiDAR-based SLAM methods and save the .PCD file into the `/map` folder.


## Run
```bash
cd your_workspace_name
source devel/setup.bash
roslaunch lmlvil xxxx.launch
```

## Example
- NTU-Viral dataset(https://ntu-aris.github.io/ntu_viral_dataset/)

<p align="center">
  <img src="https://github.com/asdleng/LiDAR-Visual-Inertial-Localization/blob/master/img/img1.gif" width="45%">
  <img src="https://github.com/asdleng/LiDAR-Visual-Inertial-Localization/blob/master/img/img2.gif" width="45%">
</p>

## Limitations

- The system requires an accurate initial pose to function correctly.

- The system is not well-suited for very large-scale environments due to its inability to maintain real-time performance.

## To Do

- Add Detailed Comments

- Organize Code Structure

