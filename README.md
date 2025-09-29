ROS2 Turtlesim Project
======================

Demo:
-----
See the turtle drawing custom shapes in the Turtlesim window (demo GIF/video should be placed in media/demo.gif).

Description:
------------
This ROS2 project demonstrates custom shape drawing with Turtlesim.
The turtle draws unique parametric shapes based on user selection.

Features:
---------
- Draw multiple unique shapes
- Adjustable scale and pen thickness
- Clears screen before drawing new shapes
- Stop and restart drawing at any time

Installation:
-------------
1. Clone the repository:
   git clone https://github.com/MalZaiter/ros2-turtlesim-project.git
   cd ros2-turtlesim-project

2. Build the workspace:
   colcon build

3. Source the setup file:
   source install/setup.bash

Running the Project:
-------------------
1. Start the Turtlesim node:
   ros2 run turtlesim turtlesim_node

2. Run the shape controller node:
   ros2 run turtle_controller turtleCommander.py

3. Publish shape commands to /selected_shape topic:
   ros2 topic pub /selected_shape std_msgs/Int32 "data: 1"

   Replace "1" with the shape ID you want to draw. 

Notes:
------
- Make sure src/log, build, and install directories are ignored via .gitignore.
- Python scripts should be executable (chmod +x turtleCommander.py) if needed.
- Works with ROS2 Humble/Foxy/Galactic (adjust commands for your version).

--------
MIT License. See LICENSE file for details.
