# Assignment 1 – ROS2 Turtle Control System (C++ & Python)

This project implements a ROS2-based multi-node control system for two turtles inside the Turtlesim simulator.
The purpose is to allow user-controlled movement of either turtle while continuously enforcing safety constraints, including collision avoidance and world-boundary checks.
The entire system is implemented both in C++ and Python.

---
## System Architecture (Text Description)

The system consists of several ROS2 nodes working together:

### UI Node (C++ or Python):
This node interacts with the user through the terminal.
It asks which turtle should be controlled and then requests linear and angular velocity values.
It publishes velocity commands for 1 second on the appropriate topic (/turtleX/cmd_vel).
It also publishes the selected turtle name on /moving_turtle.
While sending movement commands, the UI node listens on /stop_movement so it can immediately interrupt motion if needed.

### Distance Node (C++ or Python):
This node subscribes to /turtle1/pose and /turtle2/pose and continuously computes the distance between the two turtles at 10 Hz.
It also tracks which turtle is currently moving by subscribing to /moving_turtle.
If the turtles get closer than a minimum safe distance or if the moving turtle approaches the boundary of the Turtlesim world, the DistanceNode publishes zero velocities to stop the moving turtle and sends a "stop" message on /stop_movement.
It also publishes the computed distance on /turtles_distance.

### Turtle Spawner Node (Python):
Spawns turtle2 automatically at a fixed location at startup.

### Turtlesim Node:
Provides the graphical simulation environment and exposes pose and velocity topics for the turtles.

Together, these nodes create a small but complete ROS2 system with sensing, decision-making, and actuation.

--- 
## Topics Used

- /moving_turtle (std_msgs/String) UI → DistanceNode
- /stop_movement (std_msgs/String) DistanceNode → UI
- /turtles_distance (std_msgs/Float32) - DistanceNode → any subscriber
- /turtle1/cmd_vel (geometry_msgs/Twist) UI or DistanceNode → turtlesim
- /turtle2/cmd_vel (geometry_msgs/Twist) UI or DistanceNode → turtlesim
- /turtle1/pose (turtlesim/Pose) turtlesim → DistanceNode
- /turtle2/pose (turtlesim/Pose) turtlesim → DistanceNode

---
## Safety Logic

The DistanceNode ensures the system behaves safely:

### Minimum Distance Check:
If the turtles come closer than 2.0 units, the moving turtle is immediately stopped.

### Boundary Check:
If the moving turtle moves outside the safe area (x or y < 1.0 or > 10.0), the DistanceNode stops the turtle and sends a "stop" message to the UI.

### Stop Mechanism:
The DistanceNode sends zero velocity commands on /turtleX/cmd_vel
and publishes "stop" on /stop_movement.
The UI node receives this signal and immediately interrupts its 1-second movement loop.

This ensures that user commands are always overridden if unsafe.

--- 
## Building the Workspace
```
colcon build
source install/setup.bash
```
To build only the C++ package:
```
colcon build --packages-select assignment1_rt
```
To build only the Python package:
```
colcon build --packages-select assignment1_rt_py
```

## Running the System (C++ Version)

Launch simulator, spawner, and C++ DistanceNode:
```
ros2 launch assignment1_rt assignment.launch.py
```
In another terminal, run the UI node:
```
ros2 run assignment1_rt ui_node
```

## Running the System (Python Version)

Launch simulator, spawner, and Python DistanceNode:
```
ros2 launch assignment1_rt_py py_assignment.launch.py
```
In another terminal, run the UI node:
```
ros2 run assignment1_rt_py ui_py
```
## Expected Behavior

User chooses a turtle and enters linear/angular velocity.

Turtle moves for up to 1 second unless the DistanceNode triggers a stop.

The DistanceNode calculates distance and checks boundaries in real time.

When unsafe conditions occur, the turtle stops immediately.

The UI node stops sending commands as soon as it receives "stop".

System stays active for new user commands.

## Package Contents

C++ Package (assignment1_rt):

UI.cpp (interactive controller)

DistanceNode.cpp (safety node)

assignment.launch.py

Python Package (assignment1_rt_py):

ui_py.py (Python UI node)

distance_py.py (Python safety node)

py_assignment.launch.py

Turtle Spawner (turtle_spawner_py):

Spawns turtle2 automatically

## Notes

UI nodes must be run manually, because they require keyboard input.

Both C++ and Python implementations match each other in logic and comments.

This assignment demonstrates a complete multi-node ROS2 architecture
with real-time interaction and safety enforcement.