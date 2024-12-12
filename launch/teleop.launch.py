from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

# Take Joystick or keyboard inputs and publish to cmd_vel
def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    joystick_config_file = PathJoinSubstitution([
        FindPackageShare('protobot_bringup'),
        'config',
        'joystick.yaml'
    ])

    # Use the joy package to get the gamepad inputs
    # and teleop_twist_joy to convert those button presses into /cmd_vel
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joystick_config_file, {'use_sim_time': use_sim_time}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joystick_config_file, {'use_sim_time': use_sim_time}],
        # remappings=[('/cmd_vel','/cmd_vel_joy')],
    )

    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )

    # TODO: Add the option to run the keyboard node as part of this launch file
    # Use condition=IfCondition(NotSubstitution()) to toggle options, and the following node
    # to run the keyboard node in a new terminal window. Challenge is that I 
    # am getting the error [Errno 2] No such file or directory: 'xterm', and don't
    # want dependency issues when sharing with others. 
    # keyboard_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     prefix=[' xterm -e '], # Open a new terminal window to run the keyboard node
    #     condition=IfCondition(use_keyboard)
    # )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        teleop_node,
        # twist_stamper       
    ])