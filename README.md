# turtlesim_catcher


https://github.com/user-attachments/assets/0153e813-9aed-44d2-865f-3c65735fea00



This project simulates a scenario where a turtleking catches and eliminates another turtle. The turtleking also equipped with Artificial Potential Field algorithm to avoid a 'predator' as the obstacle.

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
   
## About this Repo
This project consist of 2 packages (excepting launch package)
* turtle_controller: controls turtle king's behavior. Consist of simple PID controller, communication via topics, and service client to spawn and catch other turtles.
* spawner : service server which provides ROS2 service to spawn and disappear catched turtles.

## Issues and Next Work
This project uses Artificial Potential Field (APF) as the obstacle avoidance algorithm which has several issues such as GNRON (Goal Non-Reachable with Obstacles Nearby). Next, the modified version of APF (Improved APF) will be added to solve this issue and adding detection factor to reduce turtle's pathlength when wider avoidance radius is applied.
