# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap

def generate_launch_description():
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument('prefix', default_value='""')
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    # Initialize Arguments
    use_rviz = LaunchConfiguration("use_rviz")
    prefix = LaunchConfiguration('prefix', default='""')

    # use protobot_gazebo_worlds package to start gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("protobot_gazebo_worlds"), 
                "launch",
                "world.launch.py"
            ])
        ),
        launch_arguments={
            'world_file': 'simple.sdf'
        }.items()
    )
    launch_description.add_action(gazebo_launch)

    # spawn the protobot into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "protobot",
            "-allow_renaming", "true",
            "-P", "0.0", #pitch
            "-R", "0.0", #roll
            "-Y", "0.0", #yaw
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.3"
        ],
    )
    launch_description.add_action(gz_spawn_entity)

    # Use the protobot_description package to launch robot_state_publisher, which publishes the /robot_description topic
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('protobot_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_gazebo': 'true',
            'prefix': prefix
        }.items()
    )
    launch_description.add_action(robot_state_publisher_launch)

    # # Get URDF via xacro command
    # robot_description_str = ParameterValue(
    #     Command([
    #         PathJoinSubstitution([
    #             FindExecutable(name="xacro")
    #         ]),
    #         " ",
    #         PathJoinSubstitution([
    #             FindPackageShare("protobot_description"), 
    #             "urdf", 
    #             "protobot.xacro"
    #         ]),
    #         " ",
    #         "prefix:=",
    #         prefix,
    #         " ",
    #         "use_gazebo:=",
    #         'true'
    #     ]),
    #     value_type=str
    # )
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[{
    #         "robot_description": robot_description_str,
    #         'use_sim_time': True
    #     }],
    # )
    # launch_description.add_action(robot_state_publisher_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    launch_description.add_action(joint_state_broadcaster_spawner)

    controller_config_file = PathJoinSubstitution([
        FindPackageShare("protobot_bringup"),
        "config",
        "controllers.yaml",
    ])

    # # I think the Gazebo plugin runs the controller manager
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controller_config_file],
    #     output="both",
    #     remappings=[
    #         ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
    #     ],
    # )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", controller_config_file],
    )
    launch_description.add_action(robot_controller_spawner)

    # bridge between ROS and Gazebo topics
    bridge_config_file = PathJoinSubstitution([
        FindPackageShare("protobot_bringup"),
        'config',
        'ros_gz_bridge.yaml'
    ])
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    launch_description.add_action(ros_gz_bridge_node)

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )
    launch_description.add_action(ros_gz_image_bridge)

    # bring up rviz if use_rviz is true
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("protobot_description"), 
        "rviz", 
        "protobot.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )
    launch_description.add_action(rviz_node)

    # get commands from the joystick (remapped to publish to /diff_drive_controller/cmd_vel)
    teleop_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel',dst='/diff_drive_controller/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("protobot_bringup"),
                        "launch",
                        "teleop.launch.py"
                    ])
                ),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    launch_description.add_action(teleop_launch)


    ## TODO: 
    # # cmd_vel multiplexer will take in multiple /cmd_vel topics and publish one value to the controller
    # cmd_vel_multiplexer_config_file = PathJoinSubstitution([
    #     FindPackageShare("protobot_bringup"),
    #     "config",
    #     "cmd_vel_multiplexer.yaml"
    # ])
    # cmd_vel_multiplexer_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("twist_mux"),
    #             "launch",
    #             "twist_mux_launch.py"
    #         ])
    #     ),
    #     launch_arguments={
    #         'config_topics': cmd_vel_multiplexer_config_file,
    #         'cmd_vel_out': '/diff_drive_controller/cmd_vel',
    #         'use_sim_time': 'true',
    #         'output_stamped': 'true'
    #     }.items()
    # )
    # launch_description.add_action(cmd_vel_multiplexer_launch)


    return launch_description
