"""
This examples tends to show the pre-calculation and serialization of 
trajectories. Xamla_motion offers one-to-one, one-to-many and many-to-one
calculation of trajectories.

Many-to-one is done by calling create_trajectory_cache with a target Pose and 
a SampleVolume as start. 
"""

import numpy as np

from xamla_motion import EndEffector, WorldViewClient
from xamla_motion.data_types import Pose

from xamla_motion.trajectory_caching import TaskTrajectoryCache, create_trajectory_cache

from xamla_motion import Cache

import example_utils

from example_08.sample_box_helper import get_sample_box


def get_many_to_one_trajectory_cache() -> TaskTrajectoryCache:
    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    end_effector = move_group.get_end_effector()

    world_view_folder = "example_08_trajectory_cache"

    # Get start and end from world view, switching here the meaning, since we want to go back
    end_jv = world_view_client.get_joint_values(
        "start_jv", world_view_folder)
    end_pose = end_effector.compute_pose(end_jv)
    start_pose = world_view_client.get_pose(
        "end_pose", world_view_folder)

    # Get a SampleBox, which contains a Volume defined by translations and rotations
    start_box = get_sample_box(start_pose)

    # Use a cash to store the calculated trajectories
    cache = Cache("trajectory_cache", "example_08")
    cache.load()

    key = "many_to_one"

    # First, show a one_to_one
    # Check if the key exists already in the cache
    trajectory_cache = cache.get(key)
    if trajectory_cache is None:
        print("Create the trajectory cache, since it has not been serialized yet")
        trajectory_cache = create_trajectory_cache(set_robot_service_name="/sda10f/xamlaSda10fController/set_state",
                                                   end_effector=end_effector,
                                                   seed=end_jv,
                                                   start=start_box,
                                                   target=end_pose)
        print("Serialize the calculated trajectories")
        cache.add(key, trajectory_cache)
        cache.dump()
    else:
        print("Data already serialized.")
    return trajectory_cache


if __name__ == '__main__':
    get_many_to_one_trajectory_cache()
