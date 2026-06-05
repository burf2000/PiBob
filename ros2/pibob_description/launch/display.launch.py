"""Display PiBob URDF in RViz with a joint_state_publisher_gui for manual posing."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('pibob_description')
    xacro_path = PathJoinSubstitution([pkg, 'urdf', 'pibob.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'pibob.rviz'])

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=None, output='screen'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], output='screen'),
    ])
