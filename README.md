# PKAM
## Path-Planning and Controller for Multirotor aerial vehicle.
- Controllers for controlling MAVs using the [mavros](https://github.com/mavlink/mavros) package in OFFBOARD mode. (will be a SDK)
- Path-planing with A start, RRT, steering behaviors.


## Overview
- **dmapping**: Environment representation in 3 type (pointcloud, octree, grid).
- **dcontroller**: the customize controller base-on mavros for position, atitude, raw control.
- **dplaning**: calculate way points based-on map representation (octree, grid) and generate trajectory for controller.


## New Development Note
- Add Gazebo Model animation, add PLUGIN_PATH: 
```bash
  sudo gedit ~/.bashrc
  #add this line in the end.
  export GAZEBO_PLUGIN_PATH=/home/turbo/catkin_ws/src/pkam:$GAZEBO_PLUGIN_PATH

```
- When run command : 
```bash
  roslaunch dcontroller sitl.launch 

  # for easy test
  roslaunch dcontroller sitl.launch world:=easy_wall  
  # for hard test
  roslaunch dcontroller sitl.launch world:=hard_wall
  # for dynamic test
  roslaunch dcontroller sitl.launch world:=dynamic_wall
```
- rebuild ros-wrapper: 
```bash
  cd ~/catkin_ws
  catkin clean mav_trajectory_generation_ros
  #clean mav_trajectory_generation_ros in other dir and 
  catkin build mav_trajectory_generation_ros


```


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