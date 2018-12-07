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



def callback(state):
    print("     works")
    print(state)

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
    

    get_pose = lambda : world_view_client.get_pose(point_name, world_view_folder)
    # Calculate the number of calls to jogging client based on frequency and time
    N = 200
    for i in range(N):
        if (i+1) % 10 == 0:     
            print("Call {} of {}.".format(i+1, N))
        pose = get_pose()
        jogging_client.send_set_point(pose)
        time.sleep(1/50)
    jogging_client.register(callback)
    N = 200
    for i in range(N):
        if (i+1) % 10 == 0:     
            print("Call {} of {}.".format(i+1, N))
        pose = get_pose()
        jogging_client.send_set_point(pose)
        time.sleep(1/50)
    jogging_client.unregister(callback)
    for i in range(N):
        if (i+1) % 10 == 0:     
            print("Call {} of {}.".format(i+1, N))
        pose = get_pose()
        jogging_client.send_set_point(pose)
        time.sleep(1/50)
    # Stop tracking
    jogging_client.stop()


if __name__ == '__main__':
    main()

