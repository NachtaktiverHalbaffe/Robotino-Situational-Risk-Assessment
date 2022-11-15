import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import rospkg
import os
import yaml
import xml.etree.ElementTree as ET
from tf.transformations import *

rospack = rospkg.RosPack()
rospy.init_node('insert_object',log_level=rospy.INFO)




#model_sdf_path = '/home/cpfactory/.gazebo/models/cardboard_box/model.sdf'


model_sdf_path = os.path.join(rospack.get_path("digital_twin"),'models','wall','model.sdf')



rospy.wait_for_service('gazebo/spawn_sdf_model')
print('hi')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)


'''
find and replace the pose of all models with and size value of a box 
'''
data_path = os.path.join(rospack.get_path("digital_twin"),'param','static_env.yaml')
with open(data_path, "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)
    print("Read successful")
    eParser = ET.parse(model_sdf_path)
    rootElement = eParser.getroot()
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

        for pose in rootElement.findall("./model/pose"):
            pose.text = '{} {} 0 0 0 {}'.format(round(w['center']['x']*0.015,6),round(w['center']['y']*0.015,6),w['angle']) #'xyz rpy x y 0 0 0 angle'
            print(pose.text) 
        for size in rootElement.findall("./model/link/collision/geometry/box/"):
            #print(size.text)
            size.text = '{} 0.15 2.5'.format(round(w['length']*0.015,2))  #lbhy 'length 0.15 2.5'
        for size in rootElement.findall("./model/link/visual/geometry/box/"):
            size.text = '{} 0.15 2.5'.format(round(w['length']*0.015,2))  #lbhy 'length 0.15 2.5'
            print(size.text)
        #for element in rootElement.findall("model"):#"./model/link"):
         #   print(element.set('name',wall))
        xml_str =  ET.tostring(rootElement, encoding='unicode',xml_declaration=True)
        print(xml_str)




        '''        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        f = open(model_sdf_path,'r')
        xml_str = f.read()'''
        spawn_model_prox(wall, xml_str, wall, initial_pose, "world")
        #print(xml.etree.ElementTree.tostring(e, encoding='unicode'))
    
    
