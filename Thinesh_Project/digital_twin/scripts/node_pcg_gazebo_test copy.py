#!/usr/bin/env python3

from pcg_gazebo.task_manager import Server
# First create a simulation server
server = Server()
# Create a simulation manager named default
server.create_simulation('pcg-example')
simulation = server.get_simulation('pcg-example')

simulation.create_gazebo_empty_world_task()
# A task named 'gazebo' the added to the tasks list
print('Tasks created: ', simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))

simulation.run_all_tasks()

from pcg_gazebo.generators import WorldGenerator
import random

# Create a Gazebo proxy
gazebo_proxy = simulation.get_gazebo_proxy()
print('ROS configuration:')
print(gazebo_proxy.ros_config)

from pcg_gazebo.simulation import SimulationModel

obj = SimulationModel('box')

# By changing the size, collision, visual and inertial 
# properties are already going to be updated
obj.add_cuboid_link(
    link_name='link',
    size=[0.8, 0.7, 0.9],
    mass=30)

# Print the initial state of a box in the model option
print(obj.to_sdf())


# Set default friction parameters
obj.links['link'].collisions[0].enable_property('friction')
print(obj.to_sdf('model'))

obj.links['link'].collisions[0].set_ode_friction_params(
    mu=0.9,
    mu2=0.5,
    slip1=0.3, 
    slip2=0.5,
    fdir1=[0, 0, 0]
)
print(obj.to_sdf('model'))

mu = [0.1, 0.3, 0.5, 0.7, 1.0] 
for i in range(len(mu)):
    print('\t - #{} mu = {}'.format(i, mu[i]))
    obj.links['link'].collisions[0].set_ode_friction_params(
        mu=mu[i],
        mu2=mu[i],
        slip1=0.0, 
        slip2=0.0)
    print(obj.links['link'].collisions[0].to_sdf())
    # Set a random XKCD color to each box
    obj.links['link'].visuals[0].set_xkcd_color()
    obj.name = 'box_mu_{}'.format(mu[i])
    obj.pose = [0, i, 2, 0, 0, 0]
    obj.spawn(gazebo_proxy=gazebo_proxy)

import os
import rospkg

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import rospkg
import os
import yaml
import xml.etree.ElementTree as ET
from tf.transformations import *
rospack = rospkg.RosPack()
data_path = os.path.join(rospack.get_path("digital_twin"),'param','static_env.yaml')

data_path = os.path.join(rospack.get_path("digital_twin"),'param','static_env.yaml')
with open(data_path, "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)
    print("Read successful")
    
    
    for wall in data:
        
        print(data[wall])
        w = data[wall]
        initial_pose = Pose()
        initial_pose.position.x = round(w['center']['x']*0.015,6)
        initial_pose.position.y = round(w['center']['y']*0.015,6)
        initial_pose.position.z = 0
        q = quaternion_from_euler(0,0,w['angle'])
        initial_pose.orientation.x = q[0]
        initial_pose.orientation.y = q[1]
        initial_pose.orientation.z = q[2]
        initial_pose.orientation.w = q[3]
        length = round(w['length']*0.015,2)


        from pcg_gazebo.simulation import SimulationModel

        obj = SimulationModel('box')

        # By changing the size, collision, visual and inertial 
        # properties are already going to be updated
        obj.add_cuboid_link(
            link_name=wall,
            size=[length, 0.15, 2.5])

        # Set a random XKCD color to each box
        obj.links[wall].visuals[0].set_xkcd_color()
        obj.name = 'box_mu_{}'.format(wall)
        obj.pose = [round(w['center']['x']*0.015,6), round(w['center']['y']*0.015,6), 0, 0, 0, w['angle']]
        obj.spawn(gazebo_proxy=gazebo_proxy)