# plan_env

The plan_env package, submodule of `Fast-Planner` & `EGO-Planner` & `PE-Planner`

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fplan_env.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)


## Introduction

If using "depth + (odom / pose)", set `frame_id` of map tp `map`
If using "pointcloud + odom", set `frame_id` of map tp `base_link`

### A* Search

- [occupy_map.cpp](src%2Foccupy_map.cpp)

### Fast-Planner

- [sdf_map.cpp](src%2Fsdf_map.cpp)

### EGO-Planner

- [grid_map.cpp](src%2Fgrid_map.cpp)

### PE-Planner

- [map.cpp](src%2Fmap.cpp)


## Release Note

- v1.3.0: add support for A* Search
- v1.2.1: remove support for `map_file`
- v1.2.0: add support for `PE-Planner`
- v1.1.1: remove `map_min_idx_` and `map_max_idx_`
- v1.1.0: replace `camera_q` with `camera_r`
- v1.0.4: add support for `depthOdomCallback`
- v1.0.3: Remove border inflation
- v1.0.2: add support for `cloudCallback`
- v1.0.1: Replace `/map` & `/world` with `map`


## Installation

```bash
catkin_make install --source Planning/plan_env --build Planning/plan_env/build
```
