# LiDAR-Visual-IMU Fusion Localization
![Image text](https://github.com/asdleng/LiDAR-Visual-Inertial-Localization/blob/master/img/systemoverview.png)

This project presents a robust LiDAR-Visual-Inertial locaization system running within a LiDAR-generated point cloud map, named (LM-LVIL). The code is developed based on FAST-LIVO and FAST-LIO2. The contribution of the system are as follows:
- Under the IESKF framework, the system fuses IMU integration, camera visual measurements, and LiDAR measurements within a LiDAR-built point cloud map with good synergy. 
- For visual measurements, map points are directly projected into camera frames for photometric constraints, rather than matching between the generated visual feature points and the point cloud map.
- In addition to photometric constraints in visual measurements, projection constraints, line constraints, and 3D point alignment constraints are considered to enhance the system accuracy and robustness.

![Image text](https://github.com/asdleng/LiDAR-Visual-Inertial-Localization/blob/master/img/illustrate.png)

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

## Run
```bash
cd your_workspace_name
source devel/setup.bash
roslaunch lmlvil xxxx.launch
```

## Example
- NTU-Viral dataset(https://ntu-aris.github.io/ntu_viral_dataset/)
You should build the pointcloud map using LiDAR-based SLAM methods and save the PCD map into /map folder.




