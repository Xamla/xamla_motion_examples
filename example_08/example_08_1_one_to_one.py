"""
This examples tends to show the pre-calculation and serialization of 
trajectories. Xamla_motion offers one-to-one, one-to-many and many-to-one
calculation of trajectories.

One-to-one is done by calling create_trajectory_cache with two Poses

"""

import numpy as np

from xamla_motion import EndEffector, WorldViewClient
from xamla_motion.data_types import Pose

from xamla_motion.trajectory_caching import TaskTrajectoryCache, create_trajectory_cache

import example_utils

from xamla_motion import Cache


def get_one_to_one_trajectory_cache() -> TaskTrajectoryCache:
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

    # Use a cash to store the calculated trajectories
    cache = Cache("trajectory_cache", "example_08")
    cache.load()

    key = "one_to_one"

    # First, show a one_to_one
    # Check if the key exists already in the cache
    trajectory_cache = cache.get(key)
    if trajectory_cache is None:
        print("Create the trajectory cache, since it has not been serialized yet")
        trajectory_cache = create_trajectory_cache(set_robot_service_name="/sda10f",
                                                   end_effector=end_effector,
                                                   seed=start_jv,
                                                   start=start_pose,
                                                   target=end_pose)
        print("Serialize the calculated trajectories")
        cache.add(key, trajectory_cache)
        cache.dump()
    else:
        print("Data already serialized.")
    return trajectory_cache


if __name__ == '__main__':
    get_one_to_one_trajectory_cache()
