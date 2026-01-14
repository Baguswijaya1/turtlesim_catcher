# turtlesim_catcher
https://github.com/user-attachments/assets/214da79a-dc1c-464e-ab1d-2cf25684cc93

This project simulates a scenario where a turtleking catches and eliminates another turtle. By using ROS2 and turtlesim packages, fundamentals of ROS2 and basic control system are applied.

## How to use
1. make sure ROS2 is installed. In this project, I use ROS2 Jazzy
2. Clone this repository
   ```
   git clone https://github.com/Baguswijaya1/turtlesim_catcher.git
   ```
4. make sure you are in turtlesim_catcher, then build and source
   ```
   cd turtlesim_catcher
   colcon build
   source install/setup.bash
   ```
6. launch the file
   ```
   ros2 launch turtle_launcher turtle_launcher.launch.xml
   ```
   
## Repository Explanation
This project consist of 2 packages (excepting launch package)
* turtle_controller: controls turtle king's behavior. Consist of simple P controller, communication via topics, and service client to spawn and catch other turtles.
* spawner : service server which provides ROS2 service to spawn and disappear catched turtles.
Turtlesim node, which installed together once you installed ROS2, is also used to make the platform.
