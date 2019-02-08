"""
This examples tends to show the pre-calculation and serialization of
trajectories. Xamla_motion offers one-to-one, one-to-many and many-to-one
calculation of trajectories.

Lets the user choose a joint pose in world view in the boundaries of the SampleVolume
used.
Then it uses the saved TaskTrajectoryCache to get a trajectory back and forth to the pose
"""

import numpy as np

from xamla_motion import EndEffector, WorldViewClient
from xamla_motion.data_types import Pose

from xamla_motion.trajectory_caching import TaskTrajectoryCache, create_trajectory_cache

from xamla_motion import Cache

import example_utils

from example_08.sample_box_helper import get_sample_box


def test_cached_trajectories():
    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    end_effector = move_group.get_end_effector()

    world_view_folder = "example_08_trajectory_cache"

    # Get start and end from world view
    start_jv = world_view_client.get_joint_values(
        "start_jv", world_view_folder)
    start_pose = end_effector.compute_pose(start_jv)
    end_pose = world_view_client.get_pose(
        "end_pose", world_view_folder)

    # Get a SampleBox, which cotains a Volume defined by translations and rotations
    end_box=get_sample_box(end_pose)

    # Use a cash to store the calculated trajectories
    cache=Cache("trajectory_cache", "example_08")
    cache.load()

    go_to_key="one_to_many"
    come_back_key="many_to_one"
    # First, show a one_to_one
    # Check if the key exists already in the cache
    tc_go_to=cache.get(go_to_key)
    tc_come_back=cache.get(come_back_key)
    if tc_go_to is not None and tc_come_back is not None:
        # TODO: implement the usage of the trajectories
        None
    else:
        print("Could not open serialized data. ")

if __name__ == '__main__':
    test_cached_trajectories()
