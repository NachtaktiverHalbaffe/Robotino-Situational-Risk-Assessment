# Importing the Gazebo proxy
from pcg_gazebo.task_manager import GazeboProxy

# If there is a Gazebo instance running, you can spawn the box into the simulation
from pcg_gazebo.task_manager import Server
# First create a simulation server
server = Server()
# Create a simulation manager named default
server.create_simulation('default')
print(server.is_simulation_running('gazebo_ros'))
simulation = server.get_simulation('default')
print(simulation)
# Run an instance of the empty.world scenario
# This is equivalent to run
#      roslaunch gazebo_ros empty_world.launch
# with all default parameters
if not simulation.create_gazebo_empty_world_task():
    raise RuntimeError('Task for gazebo empty world could not be created')

# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))


# Run Gazebo
simulation.run_all_tasks()


# Now create the Gazebo proxy with the default parameters. 
# If these input arguments are not provided, they will be used per default.
gazebo_proxy = simulation.get_gazebo_proxy()
# The timeout argument will be used raise an exception in case Gazebo 
# fails to start

from pcg_gazebo.simulation import create_object
from pcg_gazebo.generators import WorldGenerator

generator = WorldGenerator(gazebo_proxy)

box = create_object('box')
box.add_inertial(mass=20)
print(box.to_sdf('model'))