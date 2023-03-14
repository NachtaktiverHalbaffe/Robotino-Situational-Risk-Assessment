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
- [Basic Project Documentation](#basic-project-documentation)
  - [Installation Guide](#installation-guide)
    - [Prepare Python Nodes](#prepare-python-nodes)
  - [Manual](#manual)
    - [Run Prototype](#run-prototype)
    - [Generate Error distribution](#generate-error-distribution)
  - [Technical Docs](#technical-docs)
    - [ROS Topics](#ros-topics)

# Basic Project Documentation
## Installation Guide

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

4. It is recommended to install gazebo and jsk-recognition and check for proper installation. Either:

- Follow the link
[http://gazebosim.org/tutorials?tut=install_ubuntu&amp;cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- Use command
   ```bash
   sudo apt-get install gazebo11
   sudo apt-get install libgazebo11-dev
   sudo apt-get install ros-noetic-jsk-recognition-msgs 
   sudo apt-get install ros-noetic-jsk-rviz-plugins
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
   pip3 install -r src/Robotino/requirements.txt
   ```

### Prepare Python Nodes
To initialize Python-Scripts as nodes, they have to be made executable. For this run:
- Inside the ``/scripts`` folder to make all files executable
```zsh
chmod +x *
```
- To make one specific script executable
```zsh
chmod+x /path/to/script.py
```

## Manual
### Run Prototype
- Navigate to **workspace**
- Run ``source devel/setup.sh`` in terminal
- Start roscore:
    - If using one of the following methods, this is automatically done
    - Run ``roscore`` in terminal
- Either:
  - Start a launch configuration:
    - Run ``roslaunch prototype <launchfile>`` in terminal
      - Each launchFile has own purpose
      - prototype.launch: Launches whole prototype stack
      - autonomousOperation.launch: Launches the autonomous part of the Robotino without risk estimation
      - identifyAndMap.launch: Legacy method when running most python scripts from terminal and not as ROS Nodes
  - Start GUI: ``python3 src/robotino/mainwindow.py`` and launch ros from gui
    - This basically starts ROS with launch files in the background, so basically the same as starting with launch configuration, but gui is also started
  - Use script with automated task execution: ``python3 scriptedTaskExecution.py``
    - Follow the prompts from the terminal. Everything is explained there

### Generate Error distribution
- Make shure to have error values saved in a CSV file
- Use ``monitored_space_observer_utils.py`` to generate distributions:
  - ``generateErrorDistributionDistance()`` for generating distributions for distance (how its used) in RL agent)
  - ``generateErrorDistributionAngle()`` for generating distributions for angle
  - ``loadErrorValues()`` to just create error distribution without modifications for RL agent (distance errors e.g. are only positive)
- Use ``plotErrorDist()`` to plot the segments and probabilities (which you get from the previous mentioned functions) to generate a bar chart
- The node ``/monitored_space_observer`` does this automatically on shutdown:
  - In ``logs/error_dist_csvs/hist/`` the plots are stored
  - In ``logs/error_dist_csvs/dumps`` the error values are dumped (you have to generate the error distributions by yourself)

## Technical Docs
### ROS Topics
The prototype mainly communicates over ROS and it's Topics (implementation by Felix Brugger). Following a overview of the used topics is given:
| **Topic**                   | **Message Type**                 | **Description**                                                                                                                     |
| --------------------------- | -------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| /acml_pose                  | PoseWithCovarianceStamped        | Current position of Robotino based on LIDAR. Can be manipulated by offsets                                                          |
| /amcl_pose_source           | PoseWithCovarianceStamped        | Current position of Robotino based on LIDAR. Unmanipulated values                                                                   |
| /anomaly_detected           | Bool                             | If an anomaly was detected by the monitored space observer                                                                          |
| /bruteforce_enabled         | Bool                             | If bruteforce risk estimation should also be run                                                                                    |
| /camera_pose                | PoseWithCovarianceStamped        | Current position of Robotino based on camera only                                                                                   |
| /common_traj_markers        | MarkerArray                      | Visualization message for rviz. Shows the common paths of the other proprietary robotinos                                           |
| /custom_errodist_enabled    | Bool                             | Enables the usage of a custom error distribution specified by the corresponding path topics                                         |
| /critical_objects           | CriticalObjects (Custom message) | Visualization message for rviz for marking obstacles which have a collision risk                                                    |
| /distance_sensors           | PointCloud                       | The readings from the eight distance sensors                                                                                        |
| /emergency_break            | Bool                             | Enables a emergency break                                                                                                           |
| /freeze_objects             | Bool                             | Freezes all identified obstacles and their position (pauses object detection)                                                       |
| /image_raw                  | Image                            | Image from camera sensor                                                                                                            |
| /image_bb_ws                | Image                            | Image with the bounding boxes (workstations) from the object detection on it                                                        |
| /image_bb_moveable          | Image                            | Image with the bounding boxes (moveable obstacles) from the object detection on it                                                  |
| /lidar_breakdown            | Bool                             | If LIDAR should be in a breakdown state. Used to simulate a failing LIDAR sensor                                                    |
| /inject_offset              | Bool                             | If an offset should be injected. Currently the offset is added to /amcl_pose                                                        |
| /my_cmd_vel                 | Twist                            | Sends move commands to Robotino so it executes it                                                                                   |
| /navigation_response        | Bool                             | Response from Robotino when it ended a navigation to a node in the path                                                             |
| /nr_of_runs                 | Int16                            | The number of risk estimations for the simulative, virtual factory usecase. Also starts the correspondig risk estimation            |
| /obstacles                  | ObstacleList (Custom message)    | List of all detected obstacles                                                                                                      |
| /obstacles_geofenced        | ObstacleMsg (Custom message)     | The virtual object which is placed in the gui                                                                                       |
| /obstacle_margin            | Int16                            | Added minimum distance the path planner should keep to obstacles. Values in Pixelmap-Domain                                         |
| /obstacles_visu             | PolygonArray                     | For visualizing obstacles in rviz                                                                                                   |
| /odom                       | Odometry                         | Current odometry (linear/angular velocity and orientation) from Robotino                                                            |
| /offset                     | Float32                          | The offset which should be added to the localization if /inject_offset == True                                                      |
| /path_global                | Path                             | Global trajectory of the Robotino planned by PRM                                                                                    |
| /path_errordistr_angle      | String                           | Path where the error values of angle should be saved (CSV File)                                                                     |
| /path_errordistr_dist       | String                           | Path where the error values of distance should be saved (CSV File)                                                                  |
| /pose                       | PoseWithCovarianceStamped        | Current position of Robotino which is used by other nodes. Is  depending if LIDAR is working or noeither /acml_pose or /camera_pose |
| /pose_fallback              | PoseWithCovarianceStamped        | Starting pose which is used for path planning nd risk estimation when LIDAR breaks down or anomaly is detected                      |
| /strategy_planner_response  | String                           | Response of the strategy planner after executing a strategy                                                                         |
| /sota_enabled               | Bool                             | If the SOTA risk estimation should also be used                                                                                     |
| /target                     | Point                            | Coordinate to which the Robotino should navigate                                                                                    |
| /target_id                  | Int16                            | ID of a workstation to which the Robotino should navigate                                                                           |
| /target_markers             | MarkerArray                      | Marks the workstations in rviz                                                                                                      |
| /target_nav_markers         | MarkerArray                      | Marks the navigation points of the workstations in rviz                                                                             |
| /risk_estimation            | Risk (custom message)            | The needed parameters for risk assessment determined by the risk estimation                                                         |
| /risk_estimation_sota       | Float32                          | The risk determined by the SOTA risk estimation implemented by Abdul Rehman                                                         |
| /workstation_mapper_enabled | Bool                             | ENabled/Disables the QR Code scanner of the workstation mapper                                                                      |
