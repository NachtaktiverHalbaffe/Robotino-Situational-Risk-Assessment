# Robotino Prototype for Situative Risk Assessment

Name: Felix Brugger.
Email: st157476@stud.uni-stuttgart.de

Name: Abdul Rehman.
Email: st171920@stud.uin-stuttgart.de

Name: Kai Binder.
Email: st141569@stud.uin-stuttgart.de

## Table of Contents
- [Robotino Prototype for Situative Risk Assessment](#robotino-prototype-for-situative-risk-assessment)
  - [Table of Contents](#table-of-contents)
- [Installation Guide](#installation-guide)
  - [Prepare Python Nodes](#prepare-python-nodes)
- [Basic Project Documentation](#basic-project-documentation)
  - [Manual](#manual)
  - [Technical Docs](#technical-docs)
    - [ROS Topics](#ros-topics)
    - [Module overview (Most important ones)](#module-overview-most-important-ones)
      - [mainwindow.py](#mainwindowpy)
      - [constants.py](#constantspy)
      - [confg.py](#confgpy)
      - [PRM.py](#prmpy)
      - [object\_detection.py](#object_detectionpy)
      - [eval.py](#evalpy)
      - [PPO.py](#ppopy)
      - [Environment.py](#environmentpy)
      - [MCTS.py](#mctspy)


# Installation Guide

The prototype was developed using below configurations.

    OS: Lubuntu 20.04
    ROS: version: noetic
    Gazebo: version: 11.0

The installing system should be an Ubuntu operating system. As gazebo best used on ubuntu or a flavor of Linux with CPU that is at least an Intel I5, or equivalent. Follow the below steps to get the prototype working:

1. Install ROS
   [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Create and build a catkin workspace: Follow [ROS Wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
   or use simple command
   *“mkdir -p ~/catkin_ws/src;cd
   ~/catkin_ws/;catkin_make”*
3. Install [Robotino drivers](https://wiki.openrobotino.org/index.php?title=Robotino_OS)

   - Download below 3 files
     1. robotino-dev
     2. rec-rpc
     3. robotino-api2

   - Install the above deb files using gdebi

4. It is recommended to install gazebo and check for proper installation. Either:

- Follow the link
[http://gazebosim.org/tutorials?tut=install_ubuntu&amp;cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- Use command
   ```bash
   sudo apt-get install gazebo11
   sudo apt-get install libgazebo11-dev
   ```
5. Then install ros package gazebo_ros_pkgs. Either:
   - Follow instructions from link
      [http://gazebosim.org/tutorials?tut=ros_installing&amp;cat=connect_ros](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
   - Use packagemanager (Ubuntu only)
   ```bash
   sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
   ```

   Note:
   More information about ros wrapper(gazebo_ros_pkgs) for gazebo can be found in the link [http://gazebosim.org/tutorials?tut=ros_overview](http://gazebosim.org/tutorials?tut=ros_overview)

6. Place the project folder (IDT) in ~/catkin_ws/src
7. To use pcg-gazebo install python package with command  ```pip install pcg-gazebo```
8. Build the workspace using below commands
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   rosdep install --from-paths src --ignore-src --rosdistro noetic -y
   catkin_make --pkg robotino_msgs
   catkin_make
   ```

## Prepare Python Nodes
To initialize Python-Scripts as nodes, they have to be made executable. For this run:
- Inside the ``/scripts`` folder to make all files executable
```zsh
chmod +x*
```
- To make one specific script executable
```zsh
chmod+x /path/to/script.py
```

# Basic Project Documentation
## Manual
- Start roscore:
  - Navigate to **workspace**
  - Run ``source devel/setup.sh`` in terminal
- Start a launch configuration:
  - Run ``roslaunch prototype <launchfile>`` 
    - Each launchFile has own purpose
    - prototype.launch: Launches whole prototype stack
    - robotControl.launch: Launches the autonomous part of the Robotino
    - identifyAndMap.launch:

## Technical Docs
### ROS Topics
The prototype mainly communicates over ROS and it's Topics (implementation by Felix Brugger). Following a overview of the used topics is given:
| **Topic**            | **Message Type**              | **Description**                                                                      |
| -------------------- | ----------------------------- | ------------------------------------------------------------------------------------ |
| /acml_pose           | PoseWithCovarianceStamped     | Current position of Robotino                                                         |
| /camera_pose         | PoseWithCovarianceStamped     | Current position of Robotino based on camera only                                    |
| /image_raw           | Image                         | Image from camera sensor                                                             |
| /image_bb            | Image                         | Image with the bounding boxes from the object detection on it                        |
| /target              | Point                         | Coordinate to which the Robotino should navigate                                     |
| /target_id           | Int16                         | ID of a workstation to which the Robotino should navigate                            |
| /path_global         | Path                          | Global trajectory of the Robotino planned by PRM                                     |
| /path_local          | Path                          | Trajectory between two nodes of /path_global (e.g. if dodging a obstacle is needed ) |
| /obstacles           | ObstacleList (Custom message) | List of all detected obstacles                                                       |
| /navigation_response | Bool                          | Response from Robotino when it ended a navigation to a node in the path              |
| /odom                | Odometry                      | Current odometry (linear/angular velocity and orientation) from Robotino             |
| /my_cmd_vel          | Twist                         | Sends move commands to Robotino so it executes it                                    |
| /lidar_enabled       | Bool                          | If LIDAR should be used. Used to simulate a failing LIDAR sensor                     |
| /target_markers      | MarkerArray                   | Marks the workstations in rviz                                                       |
| /target_nav_markers  | MarkerArray                   | Marks the navigation points of the workstations in rviz                              |
### Module overview (Most important ones)
#### mainwindow.py
GUI of the prototype. It's the main file (if run with gui, see manual)
#### constants.py
File with python enumeration containing application wide constants like topic names, node names etc.
#### confg.py
Configuration file to configure agents etc with parameters.
#### PRM.py
Probabilistic Roadmap Planner. Used to create a trajectory with which the Robotino could navigate to the specified target
#### object_detection.py
Performs the object detection when using the LIDAR. Is deprecated and the camera based object detection by Kai Binder should be used
#### eval.py
Performs the situative risk assessment itself. Calculates the risk by determing the most likely loss scenario, it's probability of occurence and its cost. Is done in simulation
#### PPO.py
The Reinforcement Learning agent. It's the implementation of the adversary itself
#### Environment.py
The simulation environment in which the MAARL is performed.
#### MCTS.py
Monte Carlo Tree Search. It's used to determine the most likely loss scenario and it's probability of occurence
