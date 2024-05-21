# plan_env

The plan_env package, submodule of `Fast-Planner` & `EGO-Planner`

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FFast-Planner.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)


## Introduction

`sdf_map` is for `Fast-Planner`, `grid_map` is for `EGO-Planner`


## Release Note

- v1.1.0: replace `camera_q` with `camera_r`
- v1.0.4: support `depthOdomCallback`
- v1.0.3: Remove border inflation
- v1.0.2: support `cloudCallback`
- v1.0.1: Replace `/map` & `/world` with `map`


## Installation

```bash
catkin_make install --source src/plan_env --build build/plan_env
```
