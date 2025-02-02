# File: car2_gazebo/launch/car.launch.py

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Get the package directories
    pkg_car2_gazebo = get_package_share_directory('car2_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    xacro_file_name = "main.xacro"

    xacro_path = os.path.join(
        pkg_car2_gazebo,
        "models",
        "platforms",
        "car",
        xacro_file_name,
    ) 
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", xacro_path]),
            }
        ],
    )

    # Path to the world file
    world_file = os.path.join(pkg_car2_gazebo, 'worlds', 'empty.world')

    # Include the default Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'car'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
