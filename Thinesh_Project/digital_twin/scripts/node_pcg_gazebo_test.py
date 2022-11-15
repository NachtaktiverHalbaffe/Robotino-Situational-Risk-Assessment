#!/usr/bin/env python3


import os
import rospkg

import rospy
from gazebo_msgs.srv import SpawnModel
import rospkg
import os
import yaml
from tf.transformations import *
from pcg_gazebo.simulation import SimulationModel

rospack = rospkg.RosPack()
data_path = os.path.join(rospack.get_path("digital_twin"),'param','static_env.yaml')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

with open(data_path, "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)
    
    for wall in data:
        #print(data[wall])
        w = data[wall]

        length = round(w['length']*0.015,2)
        obj = SimulationModel('box')

        # By changing the size, collision, visual and inertial 
        # properties are already going to be updated
        obj.add_cuboid_link(
            link_name=wall,
            size=[length, 0.15, 2.5])
        x =round(w['center']['x']*0.015,6)
        y = round(w['center']['y']*0.015,6)
        yaw = w['angle']
        #print(x,y,yaw, length)
        
        # Set a random XKCD color to each box
        #obj.links[wall].visuals[0].set_xkcd_color()
        obj.links[wall].visuals[0].enable_property('material')
        obj.name = 'box_mu_{}'.format(wall)
        obj.pose = [y,x, 0, 0, 0, yaw]
        obj.static = True
        #print(obj.to_sdf('model'))
        #spawn_model_prox(wall, obj.to_sdf('model'), 'gazebo', initial_pose, "world")
        obj.spawn(gazebo_proxy=spawn_model_prox)


# done!!!  for walls
