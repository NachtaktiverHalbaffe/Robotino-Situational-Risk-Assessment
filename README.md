# Automated Evaluation of Situative Risk Assesment using Machine Learning

Name: Felix Brugger.
Email: st157476@stud.uni-stuttgart.de

### Installation Guide

The prototype was developed using below configurations.

    OS: Lubuntu 20.04

    ROS: version: noetic

    Gazebo: version: 11.0

The installing system should be an Ubuntu operating system. As gazebo best used on ubuntu or a flavor of Linux with CPU that is at least an Intel I5, or equivalent. Follow the below steps to get the prototype working.

1. Install ROS
   [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. create and build a catkin workspace: follow ROS wiki
   ([http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))
   or use simple command
   *“mkdir -p ~/catkin_ws/src;cd
   ~/catkin_ws/;catkin_make”*
3. Install Robotino drivers:
   [https://wiki.openrobotino.org/index.php?title=Robotino_OS](https://wiki.openrobotino.org/index.php?title=Robotino_OS)

Download below 3 files

1. robotino-dev
2. rec-rpc
3. robotino-api2

Install the above deb files using gdebi

4. It is recommended to install gazebo and check for proper installation.

Install gazebo: use below commands or follow the link
[http://gazebosim.org/tutorials?tut=install_ubuntu&amp;cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

*sudo apt-get install gazebo11*

*sudo apt-get install libgazebo11-dev*

5. Then install ros package gazebo_ros_pkgs. Follow the below command to install gazebo_ros_pkgs or link
   [http://gazebosim.org/tutorials?tut=ros_installing&amp;cat=connect_ros](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

*sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control*

Note:
More information about ros wrapper(gazebo_ros_pkgs) for gazebo can be found in the link [http://gazebosim.org/tutorials?tut=ros_overview](http://gazebosim.org/tutorials?tut=ros_overview)

6. Place the project folder (IDT) in ~/catkin_ws/src
7. To use pcg-gazebo install python package *pip install pcg-gazebo*
8. Build the workspace using below commands

*cd ~/catkin_ws*

*source devel/setup.bash*

*echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc*

*rosdep install --from-paths src --ignore-src --rosdistro noetic -y*

*catkin_make --pkg robotino_msgs*

*catkin_make*
