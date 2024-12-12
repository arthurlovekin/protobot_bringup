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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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
        "sim_controllers.yaml",
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

    bridge_config_file = PathJoinSubstitution([
        FindPackageShare("protobot_bringup"),
        'config',
        'ros_gz_bridge.yaml'
    ])
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            'config_file:=',
            bridge_config_file,
        ]
    )
    launch_description.add_action(ros_gz_bridge)

    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"]
    # )

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
    #     # Delay start of joint_state_broadcaster after `robot_controller`
    # # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    # start the teleop to publish to /cmd_vel
    teleop_launch = IncludeLaunchDescription(
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
    launch_description.add_action(teleop_launch)


    return launch_description


# TODO: 
# [ERROR] [1733879268.654048878] [controller_manager.resource_manager]: Caught exception of type : N9pluginlib20LibraryLoadExceptionE while loading hardware: According to the loaded plugin descriptions the class gz_ros2_control/GazeboSimSystem with base class type hardware_interface::SystemInterface does not exist. Declared types are  mock_components/GenericSystem

# https://github.com/ros-controls/ros2_control/issues/1554




# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# # Template From https://github.com/ros-controls/ros2_control_demos/blob/master/example_9/bringup/launch/rrbot_gazebo.launch.py

# def generate_launch_description():
#     launch_description = LaunchDescription()

#     # Declare arguments
#     gui_arg = DeclareLaunchArgument(
#         "gui",
#         default_value="false",
#         description="Start RViz2 automatically with this launch file.",
#     )
#     launch_description.add_action(gui_arg)

#     # Initialize Arguments
#     gui = LaunchConfiguration("gui")

#     # use protobot_gazebo package to start gazebo
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare("protobot_gazebo"),
#                 "launch",
#                 "world.launch.py"
#             ])
#         ),
#         launch_arguments={
#             'world_file': 'simple.sdf'
#         }.items()
#     )
#     launch_description.add_action(gazebo_launch)
    
#     # use protobot_description package to publish the /robot_description topic
#     # via the robot_state_publisher
#     robot_state_publisher_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('protobot_description'),
#                 'launch',
#                 'robot_state_publisher.launch.py'
#             ])
#         ]),
#         launch_arguments={
#             'use_sim_time': 'true',
#             'use_gazebo': 'true',
#             'prefix': ""
#         }.items()
#     )
#     launch_description.add_action(robot_state_publisher_launch)

#     # start the controllers
#     controller_config_file = PathJoinSubstitution([
#         FindPackageShare("protobot_bringup"),
#         "config",
#         "protobot_sim_controllers.yaml",
#     ])
#     # robot_description_str = Command(['ros2 param get --hide-type /robot_state_publisher robot_description']) #Command: ros2 param get --hide-type /robot_state_publisher robot_description
#     # Captured stderr output: Node not found
#     # I guess I'd have to sequence it to be after the robot_state_publisher

#     # control_node = Node(
#     #     package="controller_manager",
#     #     executable="ros2_control_node",
#     #     parameters=[
#     #         # {'robot_description': robot_description_str},
#     #         controller_config_file
#     #     ],
#     #     remappings=[
#     #         ("~/robot_description","/robot_description"),
#     #         # [controller_manager]: Waiting for data on 'robot_description' topic to finish initialization
#     #     ],
#     #     output="both",
#     # )
#     # launch_description.add_action(control_node)

#     robot_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["protobot_steering_controller", "--param-file", controller_config_file],
#     )
#     launch_description.add_action(robot_controller_spawner)


#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         # arguments=["joint_state_broadcaster", "--param-file", controller_config_file],
#         # arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#         arguments=["joint_state_broadcaster"],
#     )
#     launch_description.add_action(joint_state_broadcaster_spawner)

#     # use the /robot_description topic to spawn the protobot into Gazebo
#     gz_spawn_robot_node = Node(
#         package="ros_gz_sim",
#         executable="create",
#         output="screen",
#         arguments=[
#             "-topic", "/robot_description",
#             "-name", "protobot",
#             "-allow_renaming", "true",
#             "-P", "0.0", #pitch
#             "-R", "0.0", #roll
#             "-Y", "0.0", #yaw
#             "-x", "0.0",
#             "-y", "0.0",
#             "-z", "0.6"
#         ],
#     )
#     launch_description.add_action(gz_spawn_robot_node)

#     # bridge between ROS and Gazebo topics
#     ros_gz_bridge_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('ros_gz_bridge'),
#                 'launch',
#                 'ros_gz_bridge.launch.py'
#             ])
#         ),
#         launch_arguments={
#             'bridge_name': 'ros_gz_bridge',
#             'config_file':
#                 PathJoinSubstitution([
#                     FindPackageShare('protobot_bringup'),
#                     'config',
#                     'ros_gz_bridge.yaml'
#                 ])
#         }.items()
#     )
#     launch_description.add_action(ros_gz_bridge_launch)

#     # start rviz2 if gui is true
#     rviz_config_file = PathJoinSubstitution([
#         FindPackageShare("protobot_description"),
#         "rviz",
#         "protobot.rviz"
#     ])
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file],
#         condition=IfCondition(gui),
#     )
#     launch_description.add_action(rviz_node)

#     return launch_description
