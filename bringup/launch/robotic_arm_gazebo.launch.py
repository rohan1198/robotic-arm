import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "paused",
            default_value="false",
            description="Start Gazebo in paused mode.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robotic_arm"),
                    "urdf",
                    "robotic_arm_gazebo.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robotic_arm"),
            "config",
            "robotic_arm_gazebo_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robotic_arm"), "robotic_arm_description/r6bot/rviz", "view_robot.rviz"]
    )

    # Start Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'paused': paused,
            'gui': gui
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robotic_arm'
        ],
        output='screen'
    )

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Load controllers
    load_controllers = []
    load_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )
    )
    load_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotic_arm_controller", "-c", "/controller_manager"],
        )
    )

    # Define launch sequence
    nodes = [
        gazebo,
        robot_state_pub_node,
        spawn_entity,
        rviz_node,
    ]
    nodes.extend(load_controllers)

    return LaunchDescription(declared_arguments + nodes)
