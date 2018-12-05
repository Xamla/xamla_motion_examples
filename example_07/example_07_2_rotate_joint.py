"""
In this example, the jogging interface is used to rotate the robot around its 
torso joint.
"""   

import time 

from xamla_motion.data_types import JointValues, JointSet

import example_utils
from example_07.jogging_client import JoggingClient

def rotate(direction: float, jogging_client: JoggingClient, joint_name: str):
    # Creating JointValues containing exclusively 
    jv = JointValues(JointSet([joint_name]), [direction] ) 

    jogging_client.send_velocities(jv)

def main():
    torso_joint_name = example_utils.get_torso_joint_name()

    #move_group = example_utils.get_move_group()
    jogging_client = JoggingClient()
    jogging_client.set_move_group_name(example_utils.get_move_group_name())


    #Begin tracking
    jogging_client.toggle_tracking(True)

    for i in range(1000):    
        print("{} of {}".format(i, 1000))
        # First negative, becomes positive half way through
        direction = i - 500 
        rotate(direction=direction, jogging_client=jogging_client, joint_name=torso_joint_name) 
        time.sleep(0.02)

    # Stop tracking
    jogging_client.toggle_tracking(False)

if __name__ == '__main__':
    main()

