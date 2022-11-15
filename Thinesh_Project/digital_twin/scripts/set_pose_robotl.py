import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetPhysicsProperties
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

def realCallback(data):
    print(data)
    
    state_msg = ModelState()
    state_msg.model_name = 'robotino3'
    # check mapp position is needed 
    state_msg.pose.position.x = data.pose.pose.position.x - map_positon_x
    state_msg.pose.position.y = data.pose.pose.position.y - map_positon_y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = data.pose.pose.orientation.z
    state_msg.pose.orientation.w = data.pose.pose.orientation.w
    state_msg.reference_frame = 'world'
    #print(state_msg.pose.position.x)

    # wait for /gazebo/set_model_state service to set the robotino position in simulation
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        print (resp.status_message,map_positon_y)
    
    except rospy.ServiceException as e:
        print("/gazebo/set_model_state Service call failed: %s"%e)
    

if __name__ == '__main__':
    try:
        #set intial position from map, passed from rosparam in launch file.
        # else inital is set to 0 0 0, if the map file is used and not specified 
        # there will be mismatch in simulation positon

        map_pose = rospy.get_param('origin',[-1.24,-4.05,0])

        rospy.wait_for_service('/gazebo/set_model_state')

        map_positon_x = map_pose[0]
        map_positon_y = map_pose[1]
        '''
        state_msg1 = ModelState()
        state_msg1.model_name = 'robotino3'
        state_msg1.pose.position.x = map_positon_x
        state_msg1.pose.position.y = map_positon_y
        state_msg1.pose.position.z = 0
        state_msg1.pose.orientation.x = 0
        state_msg1.pose.orientation.y = 0
        state_msg1.pose.orientation.z = 0
        state_msg1.pose.orientation.w = 0
        state_msg1.reference_frame = 'world'
        # wait for /gazebo/set_model_state service to set the inital robotino position in simulation
        set_state1 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp1 = set_state1( state_msg1 )
        '''
        #print (resp.status_message)
        
        rospy.init_node('set_pose')
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, realCallback, queue_size=10)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print('error')
        pass

