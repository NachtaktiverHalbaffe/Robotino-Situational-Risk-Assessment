import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', GetModelState)


from pcg_gazebo.simulation import create_object

# If there is a Gazebo instance running, you can spawn the box into the simulation
from pcg_gazebo.task_manager import Server
# First create a simulation server
server = Server()
# Create a simulation manager named default
server.create_simulation('default')
simulation = server.get_simulation('default')
# Run an instance of the empty.world scenario
# This is equivalent to run
#      roslaunch gazebo_ros empty_world.launch
# with all default parameters
simulation.create_gazebo_empty_world_task()
# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
# Run Gazebo
simulation.run_all_tasks()


from pcg_gazebo.generators import WorldGenerator
import random
# Create a Gazebo proxy
gazebo_proxy = simulation.get_gazebo_proxy()

# Use the generator to spawn the model to the Gazebo instance running at the moment
generator = WorldGenerator(gazebo_proxy=gazebo_proxy)


box = create_object('box')
# A box object comes initially with no inertial information and null size.
print('Size:')
print(box.size)
print('Inertial:')
print(box.inertial)
print(generator.gazebo_proxy.get_model_names())

print(box.to_sdf('box'))
box.size = [0.3, 0.6, 0.2]
box.add_inertial(mass=20)

print(box.inertial)

model_counter = 0
for x in [-5, 0, 5]:
    for y in [-5, 0, 5]:
        box.visual.enable_property('material')
        box.visual.set_xkcd_color()
        generator.spawn_model(
            model=box, 
            robot_namespace='box_{}'.format(model_counter),
            pos=[x, y, 10])
        model_counter += 1

# Using the Gazebo proxy created by the generator's constructor
# it is possible to see that all models were created
print(generator.gazebo_proxy.get_model_names())


