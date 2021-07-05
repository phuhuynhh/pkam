### Install PX4 SITL(Only to Simulate)
Follow the instructions as shown in the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/
bash ./Tools/setup/ubuntu.sh
git checkout 71db090
git submodule sync --recursive
git submodule update --init --recursive
make px4_sitl_default gazebo
```
To source the PX4 environment, run the following commands

```bash
cd <Firmware_directory>
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

You can run the rest of the roslaunch files in the same terminal

```bash
cd <Firmware_directory>
DONT_RUN=1 make px4_sitl_default gazebo
```


### Installing 

Create a catkin workspace:
This folder will probably be already created since the previous process would have created it. If it is not present, do:

```bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --merge-devel
wstool init
```

### Install Realsense : 
```bash
# install realsense in ros
sudo apt-get install ros-$YOUR_ROS_VERSION$-realsense2-camera
# install decription and gazebo plugin 
# source : https://github.com/intel-ros/realsense/
cd ~/catkin_ws
catkin build realsense2_description
catkin build realsense_gazebo_plugin

```

###### Clone this repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/phuhuynh12/pkam
```
Build all the packages:

```bash
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
```

###### add this line to your ```bash ~/.bashrc ```

``` bash
# For ROS and Catkin Workspace.
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/pkam/controller/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/pkam/controller/gazebo/models

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib

#for PX4 SITL.
sh ~/catkin_ws/src/pkam/px4.sh
# Need in first time
source ~/catkin_ws/src/PX4-Autopilot/Tools/setup_gazebo.bash ~/catkin_ws/src/PX4-Autopilot ~/catkin_ws/src/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/PX4-Autopilot/Tools/sitl_gazebo

```