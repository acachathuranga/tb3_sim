## ROS2 Turtlebot3 Simulation

This package includes single and multi robot simulations for turtlebot3 in ROS2. Tested on ROS2 Humble, Ubuntu 22.04.

The simulations use slam_toolbox for mapping / localization unlike the default simulations provided with nav2_bringup package. The current version of this package uses a modified slam_toolbox package which includes additional functionalities such as online map merging. 

However, Steve Macenski's original slam_toolbox can also be used by simply changing the mapping.launch.py file given with this package. Huge Kudos to Steve for the awesome mapping package!


### Multi-Robot SLAM with turtlebot3 simulation

#### Installation

Create a workspace if you do not have one.

```shell
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws
colcon build
```
Download slam_toolbox and tb3_sim package and compile

```shell
git clone git@github.com:acachathuranga/tb3_sim.git
cd tb3_sim
git checkout v1.0

git clone git@github.com:acachathuranga/slam_toolbox.git
cd slam_toolbox
git checkout v2.0

cd ../..
colcon build --symlink-install --packages-up-to slam_toolbox tb3_sim --allow-overriding slam_toolbox 
```
Source the workspace and Launch multi-robot simulation

```shell
source ~/simulation_ws/install/local_setup.bash
ros2 launch tb3_sim multi_tb3_simulation_launch.py
```
