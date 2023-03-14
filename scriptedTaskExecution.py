import os, sys
import subprocess
import rostopic
import time
import rospy
from threading import Thread
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from utils.constants import Topics, Nodes

# Needed ROS publishers for this script
targetPub = rospy.Publisher(Topics.TARGET.value, PoseStamped, queue_size=10)
freezePub = rospy.Publisher(Topics.FREEZE_OBJECTS.value, Bool, queue_size=10)


def __printMsg(msg: str):
    """
    Prints a message to the console

    Args:
        msg (str): The message to print
    """
    lineSep = ""
    print(f"{lineSep:-^80}")
    print(msg)
    print(f"{lineSep:-^80}")


def __printMsgAndInput(msg: str, inputInfo: str):
    """
    Prints a message to the console and waits for user input to continue the script

    Args:
        msg (str): The message to print
        inputInfo (str): The message to print when waiting for an input
    """
    lineSep = ""
    print(f"{lineSep:-^80}")
    print(msg)
    input(f"{inputInfo}\n{lineSep:-^80}")


def __freezeObjects():
    """
    Freezes the identified obstacles in the environment so they don't dissapear when the camera doesn't recognize them anymore
    """
    __printMsgAndInput(
        "Make sure all obstacles are identified and located correctly. After confirmation, these obstacles are frozen for the entirety of the run",
        "Press any key if the obstacles can be frozen",
    )
    freezePub.publish(True)


def __startROS():
    """
    Starts the ROS prototype stack in another thread. Because of this,\
    this entire Python-script has to be terminated with CMD+C
    """
    launch_files_v = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "launch/", "")
    )
    launch_file = f"{launch_files_v}/prototype.launch"
    # create CLI command
    program = f"roslaunch {launch_file}"
    # Start ros via CLI
    Thread(target=subprocess.call, args=[program.split(" ")]).start()


def __executeTask(task: set):
    """
    Executes a navigation task (driving to a coordinate)

    Args:
        task (x,y-coordinate as tuple): The target where the robotino should navigate to

    Returns:
        bool: If task did execute successfully
    """
    __printMsgAndInput(
        f"Ready to start navigation task to {task}",
        f"Press any key to start navigation",
    )

    targetMsg = PoseStamped()
    targetMsg.pose.position.x = task[0]
    targetMsg.pose.position.y = task[1]

    targetPub.publish(targetMsg)

    response = rospy.wait_for_message(Topics.STRATEGY_PLANNER_RESPONSE.value, String)

    if "success" in response.data.lower():
        __printMsg(f"Successfully navigated to {task}")
        return True
    else:
        __printMsg(f"Navigation to {task} failed: {response.data}")
        return False


def main(tasks: list, continueOnError: bool = False):
    """
    It starts and setup ROS and executes the specified navigation tasks

    Args:
        tasks (list of coordinates): The coordinate to which the robotino should drive to. Must be specified in the ROS coordinate domain
        continueOnError (bool, optional): If the script should continue executing the tasks if the execution of one task fails
    """
    __printMsg("Starting ROS-Stack. Wait some time")
    time.sleep(3)

    # Start the ROS Stack
    __startROS()
    time.sleep(10)
    while True:
        try:
            rostopic.get_topic_class("/rosout")
            break
        except KeyboardInterrupt:
            return
        except:
            continue

    __printMsgAndInput(
        "ROS Started. Make sure that robotino localized itself properly",
        "Press any key to continue",
    )

    # Initialize new node before sending tasks
    rospy.init_node(Nodes.SCRIPT.value)
    # Freeze obstacles
    __freezeObjects()

    for tasks in TASKS:
        # Execute task
        successfulExecution = __executeTask(tasks)
        # Abort execution of tasks if task did fail and continueOnError == False
        if not successfulExecution and not continueOnError:
            return


if __name__ == "__main__":
    # Coordinates: Drive in a triangle in the middel of the room
    TASKS = [(-1.613, 1.198), (-2.029, 2.552), (-3.113, 1.288)]
    main(tasks=TASKS, continueOnError=False)
