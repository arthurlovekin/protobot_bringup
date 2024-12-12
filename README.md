Configuration and launch files to run the protobot

Teleop Instructions:
The joystick.yaml file determines how gamepad buttons map to cmd_vel outputs.
If you want to change this, the /joy topic tells you what buttons are being pressed
and the teleop_twist_joy node converts these button presses into cmd_vel.
On the Logitech F710 gamepad, LB is the enable button, and RB is the Enable Turbo button
No commands will be published to /cmd_vel if an enable button is not pressed. 
If you don't have a game controller, use the keyboard. In a separate terminal, run:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

Tutorials
- Config and launch file examples in [ros2_control_demos/example_9](https://github.com/ros-controls/ros2_control_demos/blob/master/example_9/bringup/config/rrbot_controllers.yaml)
- https://docs.ros.org/en/jazzy/p/joy/

TODO: 
# [controller_manager]: Waiting for data on 'robot_description' topic to finish initialization
remapping: https://github.com/ros-controls/ros2_control/issues/1262

Pass in from command directly: 
https://robotics.stackexchange.com/questions/110716/spawner-waiting-for-controller-manager-services-to-be-available-when-they-are

Copy the launch files of ros2_control_demos and don't include Launch files from other packages: https://github.com/ros-controls/ros2_control_demos/tree/master