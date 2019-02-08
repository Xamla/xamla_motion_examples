"""
This examples tends to show the pre-calculation and serialization of
trajectories. Xamla_motion offers one-to-one, one-to-many and many-to-one
calculation of trajectories.

Lets the user choose a joint pose in world view in the boundaries of the SampleVolume
used.
Then it uses the saved TaskTrajectoryCache to get a trajectory back and forth to the pose
"""

import numpy as np

import asyncio
from xamla_motion.utility import register_asyncio_shutdown_handler 


from xamla_motion import EndEffector, WorldViewClient
from xamla_motion.data_types import Pose

from xamla_motion.trajectory_caching import (TaskTrajectoryCache, 
                                            create_trajectory_cache,
                                            move_with_trajectory_cache)

from xamla_motion import Cache

import example_utils
from example_08.sample_box_helper import get_sample_box


def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a folder to world view, deletes content if existand"""

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)


def test_cached_trajectories():

    loop = asyncio.get_event_loop()
    # Register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    end_effector = move_group.get_end_effector()

    world_view_folder = "example_08_trajectory_cache"

    add_generated_folder(world_view_client, world_view_folder)

    # Get start and end from world view
    start_jv = world_view_client.get_joint_values(
        "start_jv", world_view_folder)
    start_pose = end_effector.compute_pose(start_jv)
    end_pose = world_view_client.get_pose(
        "end_pose", world_view_folder)

    # Get a SampleBox, which cotains a Volume defined by translations and rotations
    end_box = get_sample_box(end_pose)

    sample_positions = end_box.sample_positions
    for i in range(len(sample_positions[0])):
        translation = [sample_positions[0][i],
                       sample_positions[1][i],
                       sample_positions[2][i]]
        pose = Pose.from_transformation_matrix(np.eye(4))
        pose = pose.translate(translation)
        pose = pose.rotate(end_box.quaternions[0])
        # poses contains a list of poses with same translation, different rotation
        world_view_client.add_pose(element_name="pose_{}".format(i),
                                   folder_path=world_view_folder +"/generated",
                                   pose=pose)

    # Use a cash to store the calculated trajectories
    cache = Cache("trajectory_cache", "example_08")
    cache.load()
     
    go_to_key = "one_to_many"
    come_back_key = "many_to_one"
    # First, show a one_to_one
    # Check if the key exists already in the cache
    tc_go_to = cache.get(go_to_key)
    tc_come_back = cache.get(come_back_key)

    # Move to start_jv
    loop.run_until_complete(move_group.move_joints_collision_free(start_jv))

    if tc_go_to is not None and tc_come_back is not None:
        world_view_client.add_pose(element_name="target_pose",
                            folder_path=world_view_folder +"/generated",
                            pose=end_pose)
        
        
        input("Please move target_pose in world view ")
        target_pose = world_view_client.get_pose(element_name="target_pose",
                            folder_path=world_view_folder +"/generated")
        
        execution = move_with_trajectory_cache(cache=tc_go_to,
                                    end_effector=end_effector,
                                    start_joint_values=start_jv,
                                    target_pose=target_pose,
                                    max_position_diff_radius=0.05)
        loop.run_until_complete(execution)



        execution = move_with_trajectory_cache(cache=tc_come_back,
                                    end_effector=end_effector,
                                    start_joint_values=None,
                                    target_pose=start_pose,
                                    max_position_diff_radius=0.05)
        loop.run_until_complete(execution)


        # TODO: implement the usage of the trajectories
        None
    else:
        print("Could not open serialized data. ")

    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)


if __name__ == '__main__':
    test_cached_trajectories()
