# Related to issue in ros2_control repo

This is a minimal example of the problem I am having using ros2_control.

#### Steps to reproduce :

1. Create a ROS2 workspace.
2. Clone this repo in the `src` directory.
3. Also clone the [ros2_control_demos package](https://github.com/ros-controls/ros2_control_demos) in `src`.
4. Build and source the workspace.
5. Launch the ros2_control architecture with `ros2 launch minimal_example_ros2_control test_launch.launch.py`.
6. In another terminal, send commands using `ros2 topic pub -r 25 /minimal_example_controller/commands std_msgs/msg/Float64MultiArray "{data: [0, 0]}"` (you can change the publishing rate and the commands)

#### What should happen : 

In the terminal where the launch file was launched, you should see 2 different lines : 

- `[ros2_control_node-1] Commands : [0 0]`
- `[ros2_control_node-1] Received command`

After a short while, the line `[ros2_control_node-1] Received command` should stop being printed, indicating that the subscriber's callback method is not being called.

#### Expected behavior :

The subscriber's callback method is called after each new message.
