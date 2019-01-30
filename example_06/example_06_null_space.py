"""
This example shows how to rotate the torso joint while keeping the end effectors
locked at their poses. 
"""

from typing import List

import numpy as np
import math

import copy

import asyncio
import signal
import functools

from xamla_motion.world_view_client import WorldViewClient

from xamla_motion.data_types import CartesianPath, JointSet, JointValues, JointPath
from xamla_motion.motion_client import MoveGroup

from xamla_motion.utility import register_asyncio_shutdown_handler 

import example_utils

from xamla_motion import Cache

def plan_null_space_torso_move(
            left_move_group: MoveGroup, 
            right_move_group: MoveGroup, 
            full_body_start: MoveGroup,
            torso_joint_name: str,
            goal_torso_angle: float) -> List[JointValues] :
    """ 
    Calculates a list of JointValues, describing the null space torso move 

    Parameters
    ----------
    left_move_group : MoveGroup
        The MoveGroup of the left arm
    right_move_group : MoveGroup
        The MoveGroup of the right arm
    full_body_start : MoveGroup
        The The MoveGroup of the whole robot
    torso_joint_name : str
        The name of the torso joint
    goal_torso_angle : float
        The target torso angle the torso should reach

    Returns
    ----------
    List[JointValues]
        The JointValues of the movement 
    """

    # Get the JointValues of the current left and right arm configuration
    left_start = left_move_group.get_current_joint_positions()
    right_start = right_move_group.get_current_joint_positions()
    # Get the JointValues of the whole body
    full_body = full_body_start.get_current_joint_positions()

    # Get the end effectors of the left and right arm, respectively
    left_end_effector = left_move_group.get_end_effector()
    right_end_effector = right_move_group.get_end_effector()

    # Calculate poses of the end effectors
    left_start_pose = left_end_effector.compute_pose(left_start)
    right_start_pose = right_end_effector.compute_pose(right_start)
    
    # Get the current value of the torso joint
    found, start_torso_angle = full_body.try_get_value(torso_joint_name)
    assert(found)

    # Define the amount of the JointValues configurations the movement should consist of 
    N = max(10, int(abs(goal_torso_angle - start_torso_angle)/math.radians(1)))
    threshold = math.pi/4
    waypoints = []
    for i in range(N):
        last_full_body =  copy.deepcopy(full_body)

        # Calculate the current angle between start and goal
        alpha = (i - 1) / (N - 1)
        angle = (1-alpha) * start_torso_angle + alpha * goal_torso_angle

        print("Calculate index {} of {} with angle {}".format(i + 1, N, angle))

        # Now we update the JointValues of the robot by adjusting the angle of 
        # the torso joint only 
        full_body = full_body.set_values(JointSet(torso_joint_name),  [angle])

        # Calculate the inverse kinematics for the left arm and  give the 
        # altered "full_body" object (with updated angle) as seed.
        # Since the move group for the left arm does not contain the joint
        # of the torso, it can only update the joint of the arm and assumes
        # the torso joint given by the seed to be fixed
        left_tmp_jv = left_end_effector.inverse_kinematics(
                                                    pose=left_start_pose, 
                                                    collision_check=True,
                                                    seed=full_body)
        # Update the JointValues of the result of IK of the left arm 
        full_body = full_body.set_values(left_tmp_jv.joint_set, left_tmp_jv.values)

        # Do the above for the right arm
        right_tmp_jv = right_end_effector.inverse_kinematics(
                                                pose=right_start_pose, 
                                                collision_check=True,
                                                seed=full_body)

        full_body = full_body.set_values(right_tmp_jv.joint_set,  right_tmp_jv.values)
        # Assert there were no IK jumps
       # assert(2 > full_body.ik_jump_threshold detectIKJump(full_body, last_full_body, threshold)) #, 'jump IK detected')
        waypoints.append(full_body)
    return waypoints


def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a folder to world view, deletes content if existent"""

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)

async def main():
    world_view_folder = "example_06_null_space"
    world_view_client = WorldViewClient()
    # Since we want to update the Jointvalues of the  arms independently, we 
    # need the respective MoveGroups 
    left_move_group = example_utils.get_left_move_group()
    right_move_group = example_utils.get_right_move_group()
    # The MoveGroup of the whole robot
    full_body_move_group = example_utils.get_full_body_move_group() 
    # To address the torso joint in particular, we need the name
    torso_joint_name = example_utils.get_torso_joint_name()

    # Load of the begin configuration from world view. This defines the starting
    # angle of the torso 
    begin_jvs = world_view_client.get_joint_values("Begin", world_view_folder)

    print("Go to begin configuration ")
    await full_body_move_group.move_joints_collision_free(begin_jvs)


    cache = Cache("I am in a", "example_06")
    cache.load()
    waypoints = cache.get("waypoints")
    # Calculate the list of JointValues describing the movement 
    if waypoints == None:
        waypoints = plan_null_space_torso_move(left_move_group,
                                        right_move_group,
                                        full_body_move_group,
                                        torso_joint_name,
                                        0.5)
        print("Saved waypoints to file") 
        cache.add("waypoints", waypoints)
        cache.dump()
    else:
        print("Loaded waypoints from file.") 

    add_generated_folder(world_view_client, world_view_folder)
    # Save every waypoint to world view for visualization
    for i in range(len(waypoints)):
        joint_values = waypoints[i]
        name = "joint_values_{}".format( format(i, "0>2d"))
        world_view_client.add_joint_values(name, 
                            "/{}/generated".format(world_view_folder), 
                            joint_values)
          
    print("Now follow waypoints")
    # Wiggle two times
    for i in range(2):
        # Go forth ...
        joint_path = JointPath(waypoints[0].joint_set, waypoints)
        await full_body_move_group.move_joints(joint_path, velocity_scaling=0.4)
        # ... and back
        reversed_joint_path = JointPath(waypoints[0].joint_set, list(reversed(waypoints)))
        await full_body_move_group.move_joints(reversed_joint_path, velocity_scaling=0.4)

    # clean up
    world_view_client.remove_element("generated", world_view_folder)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)
    try:
        loop.run_until_complete(main())
    finally:
        loop.close()
