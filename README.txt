Custom Turtlesim ROS2 Package
=============================

This package provides a custom Turtlesim simulator and a teleoperation node for controlling turtles via keyboard.
It supports distributed operation, where the simulator runs on one computer and multiple teleop nodes can run on different computers.

Contents
--------
- simulator_node.py: Simulator node that spawns turtles, updates their positions, handles background color and sprite changes, and renders a PyGame window when at least one turtle exists.
- teleop_node.py: Teleoperation node that allows controlling a turtle using keyboard keys. It sends commands for a fixed duration and can be configured via launch parameters.
- srv/: Custom service definitions for spawning turtles, changing background color, changing turtle sprites, and killing turtles.
- launch/: Launch file for orchestrating the simulator and teleop nodes.
- resources/: Directory containing PNG sprite images for the turtles.

Usage
-----
1. Build the package:

   colcon build --packages-select custom_turtlesim
   source install/setup.bash

2. Option A: Run nodes separately (recommended for teleop with keyboard):

   Terminal 1 (Simulator):
   ros2 run custom_turtlesim simulator_node.py

   Terminal 2 (Teleop) or (Atividade_3):
   ros2 run custom_turtlesim teleop_node.py OR ros2 run custom_turtlesim atividade_3.py

   You can customize the turtle's spawn position and name using parameters:
   ros2 run custom_turtlesim teleop_node.py --ros-args -p turtle_name:=my_turtle -p x:=2.0 -p y:=8.0 -p theta:=1.57

3. Option B: Use the launch file:

   The launch file checks if the simulator is already running and starts it only if necessary.
   It also starts the teleop node in a new interactive terminal (xterm) for keyboard control.
   The teleop node receives launch arguments as parameters.

   ros2 launch custom_turtlesim custom_turtlesim.launch.py OR ros2 launch custom_turtlesim my_second_launch.launch.py

   You can customize the turtle's spawn position and name via launch arguments:
   ros2 launch custom_turtlesim custom_turtlesim.launch.py turtle_name:=launch_teleop x:=2.0 y:=8.0 theta:=1.57

Keyboard Controls (teleop)
---------------------------
- Movement (each key press triggers a fixed action):
 - W: Move forward for a short duration (stops automatically)
 - S: Move backward for a short duration (stops automatically)
 - A: Rotate left (120 degrees) for a short duration (stops automatically)
 - D: Rotate right (120 degrees) for a short duration (stops automatically)
 - Actions can be interrupted by pressing another key.

- Background colors:
 - R: Red
 - G: Green
 - B: Blue

- Sprite change:
 - T: Change turtle sprite randomly (options: ardent, bouncy, crystal, dashing, eloquent, foxy, galactic, humble, iron, jazzy, rolling)
 - 1-0, - : Change to a specific sprite (1: ardent, 2: bouncy, ..., 0: jazzy, -: rolling)

- Polygons(atividade_3.py OR my_second_launch.launch.py):
 - P: Draw polygon
 - +: Increase sides (max: 12 sides)
 - _: Decrease sides (min: 3 sides)
 - I: Increase the side length
 - K: Decrease the side length

- Draw:
 - C: clear draw

Notes
-----
- The PyGame window opens only when at least one turtle is spawned and closes when the last turtle is removed (e.g., when the teleop node stops).
- Turtle names are automatically generated if not specified via the 'turtle_name' parameter.
- Turtle spawn position (x, y, theta) can be configured via the 'x', 'y', 'theta' parameters (or launch arguments).
- The teleop node automatically kills its turtle when it stops (e.g., via Ctrl+C).
- Multiple users running teleop (on the same ROS network/domain) will spawn and control multiple turtles.
- Sprites are loaded from the 'resources' directory as PNG files and are rotated according to the turtle's orientation.
- The launch file can optionally start the teleop node in a separate terminal (xterm) for direct keyboard input and supports passing launch arguments as node parameters.
