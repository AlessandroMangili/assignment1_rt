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

# Install the package

## Prerequisites
- **ROS** must be installed on your system.
- A properly configured ROS workspace.

## 1. Download the Package
- Open a terminal.
- Navigate to the `src` folder of your ROS workspace:
  ```bash
  cd ~/catkin_ws/src
  ```
- Clone the `assignment1_rt` package repository:
  ```bash
  git clone https://github.com/AlessandroMangili/assignment1_rt
  ```

## 2. Compile the Package
- Navigate back to the main workspace directory:
  ```bash
  cd ~/catkin_ws
  ```
- Compile the package using `catkin_make`:
  ```bash
  catkin_make
  ```
- Check that the package compiled successfully. If there are no errors, a build completion message will appear.

## 3. Source the Environment
- Update the ROS environment variables to make your package accessible by adding the following line inside your `~/.bashrc` file:
  ```bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  ```
  To apply the changes, type the command:
  ```bash
  source ~/.bashrc
  ```
## 4. Run the package

### 4.1. Start the ROS Master Node
- Open a terminal and start the ROS master node (`roscore`):
  ```bash
  roscore
  ```
- Keep this terminal open.

### 4.2. Start the turtlesim simulator
- Open a new terminal and run the `turtlesim_node`:
  ```bash
  rosrun turtlesim turtlesim_node
  ```

### 4.3. Run the nodes from the `assignment1_rt` package
- Open a new terminal and start the `UI` node:
  ```bash
  rosrun assignment1_rt first_node
  ```

- Open another terminal and start the `Distance` node:
  ```bash
  rosrun assignment1_rt second_node
  ```

## 5. Interacting with the Nodes
- **UI Node (`node1`)**: Follow the terminal prompts to control the turtles.
- **Distance Node (`node2`)**: Monitors the distance between the turtles and issues warnings if they are too close or out of bounds.
