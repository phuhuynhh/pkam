# PKAM

## Path-Planning and Controller for Multirotor aerial vehicle.
- Controllers for controlling MAVs using the [mavros](https://github.com/mavlink/mavros) package in OFFBOARD mode. (will be a SDK)
- Path-planing with A start, RRT, steering behaviors.


## Overview
- **dmapping**: Environment representation in 3 type (pointcloud, octree, grid).
- **dcontroller**: the customize controller base-on mavros for position, atitude, raw control.
- **dplaning**: calculate way points based-on map representation (octree, grid) and generate trajectory for controller.


## Getting Started
### flow [Installing guide](INSTALL.md) to install dependency and build this project.

##  


## Running the code
The following launch file enables the geometric controller to follow a circular trajectory

``` bash
#For SITL and rviz display.
roslaunch dcontroller sitl.launch
#For start the controller and path_planning
  #by hand **Highly recommend**
  rosrun dcontroller dcontroller
  #in another terminal 
  rosrun dplanning dplanning

  #by launch file 
  roslaunch dcontroller controller_and_planning.launch
```

## Develop

``` bash
	#Use ROS_INFO in Dcontroller.
	#And use printf in DPlanning.


```
- [x] Moving LocalMap to mapping_node
- [ ] Occupied Trigger and APF in mapping_node
- [ ] Check occupied and APF force in planning_Node