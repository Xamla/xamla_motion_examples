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
    joggingClient = JoggingClient()

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    move_group_name = "/sda10f/right_arm_torso"
    joggingClient.set_move_group_name(move_group_name)
    #Begin tracking
    joggingClient.toggle_tracking(True)

    
    twist = Twist()
    for i in range(1000):    
        None

    # Stop tracking
    joggingClient.toggle_tracking(False)


if __name__ == '__main__':
    main()

