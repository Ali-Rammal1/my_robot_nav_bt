import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_package_dir = get_package_share_directory('my_robot_nav_bt')

    # File paths
    map_file = os.path.join(my_package_dir, 'maps', 'my_map.yaml')
    params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Gazebo world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Launch Navigation2 stack with map and params_file provided
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_nav2_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': map_file,
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # Launch your custom Behavior Tree runner node
    bt_runner_node = Node(
        package='my_robot_nav_bt',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch RViz2: If you have a custom config, it will be used;
    # otherwise, RViz will launch with default settings.
    rviz_config_file = os.path.join(my_package_dir, 'rviz', 'my_nav_view.rviz')
    if os.path.isfile(rviz_config_file):
        rviz_args = ['-d', rviz_config_file]
    else:
        rviz_args = []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'))
    ld.add_action(gazebo_launch)
    ld.add_action(nav2_launch)
    ld.add_action(bt_runner_node)
    ld.add_action(rviz_node)

    return ld
