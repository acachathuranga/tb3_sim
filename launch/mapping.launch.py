import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory("tb3_sim")

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Topic remappings
    remappings = [  #('/localized_scan', '/localized_scan_local'),
                    ('/tf', 'tf'), 
                    ('/tf_static', 'tf_static'),
                    ('/map_metadata', 'map_metadata'),
                    ('slam_toolbox/graph_visualization', 'slam_toolbox/graph_visualization_'),
                    ('/slam_toolbox/scan_visualization', 'slam_toolbox/scan_visualization'),
                    ('/slam_toolbox/clear_changes', 'slam_toolbox/clear_changes'),
                    ('/slam_toolbox/dynamic_map', 'slam_toolbox/dynamic_map'),
                    ('/slam_toolbox/manual_loop_closure', 'slam_toolbox/manual_loop_closure'),
                    ('/slam_toolbox/pause_new_measurements', 'slam_toolbox/pause_new_measurements'),
                    ('/slam_toolbox/save_map', 'slam_toolbox/save_map'),
                    ('/slam_toolbox/serialize_map', 'slam_toolbox/serialize_map'),
                    ('/slam_toolbox/toggle_interactive_mode', 'slam_toolbox/toggle_interactive_mode')]

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_robot_name_argument = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot Name / Namespace')

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='multirobot_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        remappings=remappings + [('/map', 'local_map')],
        parameters=[
          ParameterFile(os.path.join(pkg_dir, 'config', 'mapper_params.yaml'), allow_substs=True),
          {'use_sim_time': use_sim_time}],
        # prefix=['xterm -e gdb -ex run --args']
        )
    
    start_async_slam_toolbox_merger_node = Node(
        package='slam_toolbox',
        executable='multirobot_slam_toolbox_node',
        name='slam_toolbox_merger',
        namespace=namespace,
        output='screen',
        remappings=remappings + [('/map', 'map')],
        parameters=[
          ParameterFile(os.path.join(pkg_dir, 'config', 'mapper_params.yaml'), allow_substs=True),
          {'use_sim_time': use_sim_time}],
        # prefix=['xterm -e gdb -ex run --args']
        )
    
    start_map_transform_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('map_transform_publisher'), 'launch', 'map_transform_publisher.launch.py')),
        launch_arguments={
                            'robot_name': namespace,
                            'simulation': use_sim_time
                            }.items())
 
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_argument)
    ld.add_action(declare_use_sim_time_argument)
    
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(start_async_slam_toolbox_merger_node)
    # ld.add_action(start_map_transform_publisher_node)

    return ld
