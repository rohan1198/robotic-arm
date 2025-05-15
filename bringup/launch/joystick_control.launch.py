from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Device for joystick'),

        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]),

        # Joystick control node
        Node(
            package='robotic_arm',
            executable='joystick_control.py',
            name='joystick_control',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate': 50.0,
                'max_velocity': 1.0,
                'linear_x_axis': 1,   # Left stick Y
                'linear_y_axis': 0,   # Left stick X
                'angular_z_axis': 2,  # Right stick X
                'joint_4_axis': 3,    # Right stick Y
                'joint_5_axis': 6,    # D-pad X
                'joint_6_axis': 7,    # D-pad Y
                'deadman_button': 4,  # L1 button
                'scale_linear': 0.5,
                'scale_angular': 0.5,
                'scale_joint': 0.5
            }]),
    ])
