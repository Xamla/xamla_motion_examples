"""
This example shows how to rotate the torso joint while keeping the end effectors
at certain poses 
"""

import numpy as np
import math

import time 

import copy



from xamla_motion.world_view_client import WorldViewClient

from xamla_motion.data_types import Pose, JointSet, JointValues, JointPath
from xamla_motion.motion_client import MoveGroup


import example_utils

from example_07.jogging_client import JoggingClient, Twist



def main():
    world_view_folder = ""
    jogging_client = JoggingClient()

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    move_group_name = "/sda10f/right_arm_torso"
    jogging_client.set_move_group_name(move_group_name)
    #Begin tracking
    jogging_client.toggle_tracking(True)

    # Go back and forth in x direction (linear velocity)
    forth_twist = Twist(linear=[0.5,0,0], angular=[0,0,0])
    back_twist = Twist(linear=[-0.5,0,0], angular=[0,0,0])

    N=100
    for i in range(N):    
        print("Call {} of {}".format(i+1, N))
        jogging_client.send_twist(forth_twist)
        time.sleep(0.02)
    for i in range(N):    
        print("Call {} of {}".format(i+1, N))
        jogging_client.send_twist(back_twist)
        time.sleep(0.02)


    twist2 = Twist([0,0,0], [4,5,6])
    # Stop tracking
    jogging_client.toggle_tracking(False)


if __name__ == '__main__':
    main()

