# Assignment 1 - Robot Control and Distance Monitoring

## Overview
This project involves creating a ROS package named `assignment1_rt`, which includes two nodes that control and monitor two turtles. The tasks are divided into two main nodes:

1. **UI (Node1)**: A simple user interface for controlling the velocity of the turtles.
2. **Distance (Node2)**: A distance monitoring system that checks the relative distance between the turtles and prevents collisions or out-of-bound movements.

## Nodes Description

### 1. UI Node (node1)
This node provides a user interface for controlling the turtles. The user can select a turtle (`turtle1` or `turtle2`) and set its velocity. After one second, the turtle will stop moving.

**Features:**
- **Turtle Selection:** Allow the user to select between two turtles (`turtle1` or `turtle2`).
- **Velocity Control:** The user can input the desired velocity for the selected turtle.
- **Movement Duration:** The turtle will move for 1 second with the specified velocity and then stop.
- **Repetition:** After the turtle stops, the user can input a new command.

### 2. Distance Node (node2)
This node monitors the relative distance between `turtle1` and `turtle2`. It checks if the turtles are too close to each other or if either turtle is close to the environment boundaries.

**Features:**
- **Distance Measurement:** Continuously monitor and publish the distance between `turtle1` and `turtle2` using a `std_msgs/Float32` message.
- **Collision Prevention:** If the distance between the turtles becomes too small (based on a predefined threshold), stop the moving turtle.
- **Boundary Monitoring:** If either turtle approaches the boundaries of the environment stop its movement.