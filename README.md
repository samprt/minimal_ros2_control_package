# Related to issue in ros2_control repo

This is a minimal example of the problem I am having using ros2_control.

#### Files in the repo :

- minimal_example_system(.hpp/.cpp) : SImple implementation of a ros2_control hardware system, that controls 2 joints.
- minimal_example_controller(.hpp/.cpp) : Implementation of a very basic ros2 controller that subscribes to a command topic and forwards the commands to the hardware system.
- config/controllers.yaml : Config for the minimal_example_controller.
- launch/test_launch.launch.py : ROS2 launch file

You can enable and disable the use of `asio` by commenting [this  line](https://github.com/samprt/minimal_ros2_control_package/blob/bd1249f0dabe769e8111639e3ba8498417886190/include/minimal_example_ros2_control/minimal_example_system.hpp#L19) in the minimal_example_system.hpp file.

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
