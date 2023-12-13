# ROS2 Playground Repository

Welcome to the ROS2 Playground Repository! This repository is designed to help you dive into ROS2, featuring Python and C++ packages that serve as an introduction to ROS2, showcasing the basic use of elementary ROS functions. The highlight of this repository is a turtlesim mini project, where a turtle navigates a simulation using Proportional Control to collect other turtles.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Folder Structure](#folder-structure)
- [Contributing](#contributing)
- [License](#license)

## Installation

To get started with this repository, follow these steps:

1. Clone the repository:

    ```bash
    git clone https://github.com/your-username/ros2-intro.git
    cd ros2-intro
    ```

2. Build the ROS2 workspace:

    ```bash
    colcon build
    ```

3. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Usage

### Running ROS2 Nodes

1. Launch the turtlesim mini project:

    ```bash
    ros2 launch my_robot_bringup turtlesim_miniproject.launch.py
    ```

2. Explore and interact with the turtles in the simulation.

### Running Python Nodes

1. Run a Python node from `my_py_pkg`:

    ```bash
    ros2 run my_py_pkg my_first_node.py
    ```

2. Check the console output for the node's behavior.

### Running C++ Nodes

1. Build the C++ packages:

    ```bash
    colcon build --packages-select my_cpp_pkg
    ```

2. Run a C++ node from `my_cpp_pkg`:

    ```bash
    ros2 run my_cpp_pkg my_first_node
    ```

3. Observe the node's functionality.

## Folder Structure

- `.gitignore`: Gitignore file to exclude certain files from version control.
- `src/`: Source code for ROS2 packages.
  - `my_cpp_pkg/`: C++ packages for ROS2.
  - `my_py_pkg/`: Python packages for ROS2.
  - `my_robot_bringup/`: Launch files for bringing up robot functionalities.
  - `my_robot_interfaces/`: Message and service definitions.
  - `my_turtle_sim_project/`: Python packages for the turtlesim mini project.
  - `vscode/`: VSCode configuration files.
 
## Acknowledgment

This repository was created while following and coding along with the tutorials provided by [Robotics Backend](https://roboticsbackend.com).

Feel free to explore ROS2 with this repository, and happy coding!
