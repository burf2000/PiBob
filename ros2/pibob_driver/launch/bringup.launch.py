"""Bring up robot_state_publisher + servo_bridge for live PiBob control.

By default subscribes to /joint_commands (remap from /joint_states if you want
the GUI sliders to drive the robot directly).
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare('pibob_description')
    drv_pkg = FindPackageShare('pibob_driver')

    xacro_path = PathJoinSubstitution([desc_pkg, 'urdf', 'pibob.urdf.xacro'])
    joint_map = PathJoinSubstitution([drv_pkg, 'config', 'joint_map.yaml'])

    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),

        Node(package='pibob_driver', executable='servo_bridge',
             name='servo_bridge', parameters=[joint_map], output='screen'),
    ])
