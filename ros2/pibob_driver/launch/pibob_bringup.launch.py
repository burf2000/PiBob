"""PiBob bring-up: RViz + joint sliders, with optional real-robot control.

Always starts:
  - robot_state_publisher  (URDF from pibob_description)
  - joint_state_publisher_gui  (sliders -> /joint_states)
  - rviz2  (pibob.rviz config)

So you can pose the model in RViz with the sliders. Add `control_real_robot:=true`
to ALSO drive the physical PiBob: the servo_bridge subscribes to the same
/joint_states the sliders publish and POSTs each joint to the Flask /set API.

Examples:
  # sim only (RViz + sliders, robot untouched)
  ros2 launch pibob_driver pibob_bringup.launch.py

  # sim + real robot
  ros2 launch pibob_driver pibob_bringup.launch.py control_real_robot:=true

  # point at a specific PiBob (use the IP from inside Docker; mDNS .local
  # does not resolve in the container)
  ros2 launch pibob_driver pibob_bringup.launch.py \
       control_real_robot:=true base_url:=http://192.168.20.227
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare('pibob_description')
    drv_pkg = FindPackageShare('pibob_driver')

    xacro_path = PathJoinSubstitution([desc_pkg, 'urdf', 'pibob.urdf.xacro'])
    rviz_config = PathJoinSubstitution([desc_pkg, 'rviz', 'pibob.rviz'])
    joint_map = PathJoinSubstitution([drv_pkg, 'config', 'joint_map.yaml'])

    # on_stderr='warn': xacro can emit harmless warnings (e.g. "redefining
    # global symbol: pi"); don't let those abort the launch. We still get the
    # URDF on stdout. (The URDF itself is a work-in-progress.)
    # ParameterValue(..., value_type=str): the URDF is XML, not YAML — without
    # this, launch tries to parse it as YAML and fails.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path], on_stderr='warn'), value_type=str)
    }

    control_real_robot = LaunchConfiguration('control_real_robot')
    base_url = LaunchConfiguration('base_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_real_robot', default_value='false',
            description='If true, also POST joint angles to the physical PiBob.'),
        DeclareLaunchArgument(
            'base_url', default_value='http://192.168.20.227',
            description='PiBob Flask base URL. Use the IP from inside Docker '
                        '(mDNS .local does not resolve in the container).'),

        # --- Visualisation (always on) ---
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], output='screen'),

        # --- Real robot bridge (opt-in) ---
        # The sliders publish /joint_states; remap the bridge's input onto it so
        # the same motion that moves the RViz model also moves the servos.
        Node(package='pibob_driver', executable='servo_bridge', name='servo_bridge',
             output='screen',
             parameters=[joint_map, {'base_url': base_url}],
             remappings=[('joint_commands', 'joint_states')],
             condition=IfCondition(control_real_robot)),
    ])
