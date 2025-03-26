#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os

def check_files():
    try:
        # Get package directories
        tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
        tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
        my_package_dir = get_package_share_directory('my_robot_nav_bt')
    except Exception as e:
        print("Error finding one of the packages:", e)
        return

    # Construct file paths
    gazebo_launch_path = os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
    nav2_launch_path   = os.path.join(tb3_nav2_dir, 'launch', 'navigation2.launch.py')
    map_file           = os.path.join(my_package_dir, 'maps', 'my_map.yaml')
    rviz_config_file   = os.path.join(my_package_dir, 'rviz', 'my_nav_view.rviz')

    # Print out paths and whether they exist
    print("Gazebo launch file path:")
    print("  ", gazebo_launch_path)
    print("  Exists?", os.path.isfile(gazebo_launch_path))
    
    print("\nNavigation2 launch file path:")
    print("  ", nav2_launch_path)
    print("  Exists?", os.path.isfile(nav2_launch_path))
    
    print("\nMap file path:")
    print("  ", map_file)
    print("  Exists?", os.path.isfile(map_file))
    
    print("\nRViz config file path:")
    print("  ", rviz_config_file)
    print("  Exists?", os.path.isfile(rviz_config_file))

if __name__ == "__main__":
    check_files()
