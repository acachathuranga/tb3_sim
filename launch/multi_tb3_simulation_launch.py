# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example for spawning multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import TimerAction


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('tb3_sim')
    launch_dir = os.path.join(package_dir, 'launch')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 4.0, 'y_pose': -10.0, 'z_pose': 0.01,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot2', 'x_pose': -2.0, 'y_pose': -10.0, 'z_pose': 0.01,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        ]

    # # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')
    rviz_config_file = LaunchConfiguration('rviz_config')
    log_settings = LaunchConfiguration('log_settings', default='True')

    # # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # default_value=os.path.join(package_dir, 'urdf', 'world_only.model'),
        #default_value=os.path.join(package_dir, 'worlds', 'map2.world'),
        default_value=os.path.join(package_dir, 'worlds', 'willow_garage_closed.world'),
        # default_value=os.path.join(package_dir, 'worlds', 'willow_garage.world'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'rviz', 'namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    # Start Gazebo with plugin providing the robot spawning service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        nav_instances_cmds.append(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_dir,
                                                           'launch',
                                                           'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'use_sim_time': 'True',
                                  'rviz_config_file': os.path.join(package_dir, 'rviz', 'namespaced_view.rviz'),
                                  'use_simulator': 'False',
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items()),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
        ], scoped=False))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
