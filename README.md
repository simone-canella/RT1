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

--------------------

# Assignment 2 – ROS2 Mobile Robot Safety Controller (C++)

This assignment implements a ROS2-based control and safety system for a mobile robot
inside a provided 3D simulation environment.

The goal is to allow user-driven motion while enforcing safety constraints based on
laser scanner data, and to expose runtime configurability and monitoring using
custom ROS messages and services.

The entire system is implemented in C++ using ROS2.

---

## System Architecture

The system is composed of **two ROS2 nodes**:

### 1. Teleop Interface Node (`teleop_interface`)
This node provides a CLI (Linux-Command-Line-Interface) that allows the user to:

- Insert linear velocity
- Insert angular velocity
- Specify the duration of the command

The node publishes velocity commands on the topic `/user_cmd_vel` at 10 Hz
for the specified duration, and automatically sends a zero-velocity command
when the command ends.

This node does not perform any safety checks.

### 2. Safety Controller Node (`safety_controller`)
This node is the core of the system and is responsible for:

- Receiving user velocity commands
- Monitoring laser scan data
- Monitoring robot odometry
- Enforcing safety constraints
- Handling recovery behavior
- Providing ROS services and custom messages

The node decides whether user commands can be forwarded to the robot
or whether a recovery maneuver must be executed.

---

## Topics Used

### Subscribed Topics
- `/user_cmd_vel` (`geometry_msgs/Twist`)
  User velocity commands from the teleop node

- `/scan` (`sensor_msgs/LaserScan`)
  360° laser scanner data

- `/odom` (`nav_msgs/Odometry`)
  Robot position and orientation

  ### Published Topics
- `/cmd_vel` (`geometry_msgs/Twist`)
  Final velocity command sent to the robot

- `/obstacle_info` (`assignment2_rt_cpp/msg/ObstacleInfo`)
  Custom message containing information about the closest obstacle

---

## Custom Message

### `ObstacleInfo.msg`
This message is published continuously by the safety controller and contains:

- Distance to the closest obstacle
- Direction of the obstacle
- Current safety threshold

```
float32 closest_distance
uint8 direction
float32 threshold
```

### Direction Encoding
- `0` → FRONT
- `1` → LEFT
- `2` → BACK
- `3` → RIGHT

---

## Services

Two ROS services are provided by the safety controller node.

### 1. Set Threshold Service

Service name:
`/set_threshold`

This service allows the user to change the minimum allowed distance to obstacles
during runtime.

```
float32 new_threshold
---
bool success
float32 current_threshold
```

Rules:
- Threshold values ≤ 0 are rejected
- On rejection, the previous threshold is preserved

### 2. Get Last 5 Inputs Average Service

Service name:
`/get_last5_input_avg`

This service returns the average linear and angular velocity
of the **last 5 user commands**.

```
---
float32 avg_linear_x
float32 avg_angular_z
uint8 count
```

If fewer than 5 commands are available, the average is computed
using the available samples.

---

## Safety Logic and Recovery Behavior

The safety controller continuously evaluates laser scan data while
user commands are active.

### Obstacle Detection
- The minimum distance in the laser scan is computed
- The angle corresponding to the closest obstacle is classified into
  FRONT, LEFT, BACK, or RIGHT

### Recovery Trigger
Recovery mode is activated when:
- A user command is currently active
- The robot is moving
- The closest obstacle distance is less than or equal to the threshold

### Recovery Behavior
When recovery mode is triggered:
- User commands are ignored
- The robot is commanded back to the last known safe position
- A proportional controller is used for linear and angular motion
- The robot stops once the safe position is reached

### Safe Position Definition
- The initial safe position is the startup pose
- After each user command that ends **without entering recovery mode**,
  the current pose becomes the new safe position

---

## Parameters and Configuration

All parameters of the safety controller are stored in a YAML configuration file.

### Configuration File
`config/safety_controller.yaml`

```
safety_controller:
  ros__parameters:
    threshold: 0.8
    debug: true
    debug_period: 1.0
```

Parameters can be changed without recompiling the code.

---

## Launch File

A dedicated launch file is provided to start the safety controller
with its configuration.

### Launch File
`launch/safety_controller.launch.py`

This launch file starts:
- `safety_controller` node
- Loads parameters from the YAML file

The teleop node is intentionally **not launched automatically**
and should be started manually by the user, since it is a CLI.

---

## Building the Package

From the ROS2 workspace root:

```
colcon build --packages-select assignment2_rt_cpp
source install/setup.bash
```
---

## Running the System

### 1. Start the Simulation Environment
(Provided by the professor)

### 2. Launch the Safety Controller
```
ros2 launch assignment2_rt_cpp safety_controller.launch.py
```

### 3. Run the Teleop Interface
```
ros2 run assignment2_rt_cpp teleop_interface
```

---

## Testing Features

### Change Threshold
```
ros2 service call /set_threshold assignment2_rt_cpp/srv/SetThreshold "{new_threshold: 1.2}"
```

### Get Average of Last 5 Commands
```
ros2 service call /get_last5_input_avg assignment2_rt_cpp/srv/GetLast5InputAvg "{}"
```

### Monitor Obstacle Information
```
ros2 topic echo /obstacle_info
```

---
## Notes and Design Choices

- Safety logic is centralized in a single node
- Recovery behavior is deterministic and repeatable
- Custom messages and services improve modularity and introspection
- Parameters are externalized using YAML files
- Extensive logging is available via debug parameters

This assignment demonstrates a complete ROS2 system integrating:
- Sensing 
- Control  
- Safety
- Services
- Configuration managemen

--------------------

# Assignment 1 (RT2) – ROS2 Robot Target Action System (C++)

This assignment implements a ROS2 Action-based control system for a differential drive robot (**mogi_bot**) inside the Gazebo 3D simulation environment.

The goal is to provide an interactive interface where a user can send target coordinates (x, y, theta) to an action server. The server handles navigation using proportional control logic and enforces a robust "hard-brake" sequence to ensure the robot stops precisely at the requested goal without simulation drift.

The entire system is implemented in C++ and utilizes multi-threading to allow concurrent user interaction and real-time motion control.

---

## System Architecture

The system is composed of **two ROS2 nodes** and a communication bridge:

### 1. Action Server Node (`action_server_node`)
This node is the core of the navigation logic and is responsible for:
- Implementing the Action Server for the `RobotTarget` interface.
- Monitoring the robot position via **TF2** transforms (from `odom` to `base_footprint`).
- Calculating Euclidean distance and heading error to the goal.
- Implementing an **atomic state-lock** (`is_busy_`) to prevent concurrent goal execution (Ghost Thread Protection).
- Executing a proportional control loop and a **burst-stop** sequence once the goal is reached.

### 2. Action Client Node (`action_client_node`)
This node provides the user interface and is responsible for:
- Providing a CLI that allows the user to input coordinates (x, y, theta).
- Utilizing a **dedicated UI thread** to handle terminal input without blocking ROS 2 communication.
- Synchronizing the user prompt with server results using atomic flags.
- Validating user input to ensure robust operation.

### 3. ROS-Gazebo Bridge (`ros_gz_bridge`)
Acts as the communication layer between ROS 2 and Gazebo, bridging:
- `/tf` for localization data.
- `/odom` for odometry tracking.
- `/cmd_vel` for robot actuation.

---

## Action Interface

### `RobotTarget.action`
The system uses a custom action definition to manage goal-oriented movement.

**Goal**
- `float32 x`
- `float32 y`
- `float32 theta`

**Result**
- `bool success`

**Feedback**
- `float32 dist_x`
- `float32 dist_y`

---

## Topics Used

- `/tf` (`tf2_msgs/msg/TFMessage`) ROS 2 ↔ Gazebo (Localization)
- `/cmd_vel` (`geometry_msgs/msg/Twist`) Server → Gazebo (Actuation)
- `/odom` (`nav_msgs/Odometry`) Gazebo → ROS 2 (Odometry reference)

---

## Navigation and Safety Logic

The server ensures precise navigation and prevents common simulation errors:

### Proportional Control:
A proportional controller calculates the robot speed based on error:
- **Linear Velocity:** Proportional to the distance remaining to the goal.
- **Angular Velocity:** Proportional to the heading error, normalized between [-pi, +pi].

### Ghost Thread Protection:
The server implements a gatekeeper mechanism. If a goal is already being executed, new requests are explicitly **REJECTED**. This prevents multiple control loops from conflicting over the robot's motors.

### Hard-Brake Mechanism:
To overcome "sticky" velocity commands in simulation, the server executes a stop sequence:
- Detects when the distance is within the `THRESHOLD` (0.1).
- Immediately publishes an explicit zero-velocity message.
- Executes a burst of **10 stop commands** over 200ms to clear the velocity buffer.

---

## Building the Package

From the ROS2 workspace root:
```
colcon build --packages-select assignment1_rt2 assignment1_rt2_interfaces
source install/setup.bash
```

---

## Running the System

1. **Start Gazebo:** Launch the simulation environment containing the `mogi_bot`.
  ```
  ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
  ```
2. **Launch Server and Bridge:**
  ```
   ros2 launch assignment1_rt2 assignment_launch.py
  ```
3. **Run the Action Client:***
  ```
  ros2 run assignment1_rt2 action_client_node
  ```
---

### 6. Expected Behavior
```markdown
## Expected Behavior

- **Interactive Input:** The user enters coordinates as a single line (e.g., `2.0 1.5 0.0`).
- **Real-time Feedback:** The robot moves towards the target while providing real-time distance feedback.
- **Safety Lock:** The server rejects new goals if the robot is currently in motion to prevent conflicting threads.
- **Hard Stop:** Upon reaching the goal, the robot executes a hard stop burst to ensure it remains stationary.
---

## Notes and Design Choices

- **TF2 Localization:** Used `lookupTransform` for higher precision and stability within the Gazebo environment compared to the raw `/odom` topic.
- **Thread Safety:** Implemented `std::atomic` flags for synchronization between the UI thread and the main ROS 2 callback thread.
- **State Management:** An `is_busy_` flag in the server ensures that navigation logic remains deterministic and single-threaded per goal.

This assignment demonstrates a complete ROS 2 Action architecture integrated with a 3D simulation environment and multi-threaded user interaction.