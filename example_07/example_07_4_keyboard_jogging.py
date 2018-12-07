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

from example_07.example_07_keyboard_control import JoggingInterface



def main():
    world_view_folder = "example_07_jogging"
    jogging_client = JoggingClient()

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    move_group_name = "/sda10f/right_arm_torso"
    jogging_client.set_move_group_name(move_group_name)
    #Begin tracking
    jogging_client.start()
    point_name = "TrackingPose"
    # Write pose to World View, so the tracking begins with the current end effector pose
    world_view_client.update_pose( point_name,  world_view_folder, current_pose)
    
    interface = JoggingInterface(jogging_client)


    interface.thread_start()

    # Stop tracking
    jogging_client.stop()



if __name__ == '__main__':
    main()

