# plan_env

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fplan_env.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

The plan_env package, submodule of `Fast-Planner` & `EGO-Planner` & `PE-Planner`

### A* Search

- [occupy_map.cpp](src%2Foccupy_map.cpp)

### Fast-Planner

- [sdf_map.cpp](src%2Fsdf_map.cpp)

### EGO-Planner

- [grid_map.cpp](src%2Fgrid_map.cpp)

默认是按照深度相机方案对环境进行重建，但如果是激光雷达方案，请手动修改`grid_map.cpp`中的这部分代码：

将122行到127行注释：

```cpp
odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/odom", 100));
sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
```

将132行到135行取消注释：

```cpp
odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/odom", 100));
sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
```

### PE-Planner

- [map.cpp](src%2Fmap.cpp)

## Installation

```bash
git clone https://gitee.com/hyx020222/plan_env.git ~/easondrone_ws/reconstruct/plan_env
cd ~/easondrone_ws && catkin_make --source reconstruct/plan_env --build reconstruct/plan_env/build
```
