"""
This example shows TODO: 
"""
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


def plan_null_space_torso_move(
            left_move_group: MoveGroup, 
            right_move_group: MoveGroup, 
            full_body_start: MoveGroup,
            torso_joint_name: str,
            goal_torso_angle: float):
    left_end_effector = left_move_group.get_end_effector()
    right_end_effector = right_move_group.get_end_effector()

    # get the joint values of the positions
    left_start = left_move_group.get_current_joint_positions()
    right_start = right_move_group.get_current_joint_positions()

    # Calculate poses
    left_start_pose = left_end_effector.compute_pose(left_start)
    right_start_pose = right_end_effector.compute_pose(right_start)
    
    full_body = full_body_start.get_current_joint_positions()

    found, start_torso_angle = full_body.try_get_value(torso_joint_name)
    assert(found)

    N = max(10, int(abs(goal_torso_angle - start_torso_angle)/math.radians(1)))
    threshold = math.pi/4
    waypoints = []
    for i in range(N):
        last_full_body =  copy.deepcopy(full_body)
        print("Calculate index {} of {} ".format(i + 1, N))
        # last_full_body = full_body:clone()
        alpha = (i - 1) / (N - 1)
        angle = (1-alpha) * start_torso_angle + alpha * goal_torso_angle
        print(angle)
        full_body = full_body.set_values(JointSet(torso_joint_name),  [angle])
        print(full_body)

        left_tmp_jv = left_end_effector.inverse_kinematics(
                                                    pose=left_start_pose, 
                                                    collision_check=True,
                                                    seed=full_body)
        print(left_tmp_jv)
     
        full_body = full_body.set_values(left_tmp_jv.joint_set, left_tmp_jv.values)

        right_tmp_jv = right_end_effector.inverse_kinematics(
                                                pose=right_start_pose, 
                                                collision_check=True,
                                                seed=full_body)

        full_body = full_body.set_values(right_tmp_jv.joint_set,  right_tmp_jv.values)
  #      print(full_body)
       # assert(2 > full_body.ik_jump_threshold detectIKJump(full_body, last_full_body, threshold)) #, 'jump IK detected')
        waypoints.append(full_body)
    return waypoints


def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a folder to world view, deletes content if existand"""

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)

async def main():
    world_view_folder = "example_06_null_space"
    world_view_client = WorldViewClient()


    left_move_group = example_utils.get_left_move_group()
    right_move_group = example_utils.get_right_move_group()

    full_body_move_group = MoveGroup("/sda10f")

    torso_joint_name = "torso_joint_b1"

    begin_jvs = world_view_client.get_joint_values("Begin", world_view_folder)

    print("Go to begin configuration ")
    await full_body_move_group.move_joints_collision_free(begin_jvs)

    waypoints = plan_null_space_torso_move(left_move_group,
                                        right_move_group,
                                        full_body_move_group,
                                        torso_joint_name,
                                        -0.5)

    add_generated_folder(world_view_client, world_view_folder)
    # save every waypoint to world view 
    for i in range(len(waypoints)):
        joint_values = waypoints[i]
        name = "joint_values_{}".format( format(i, "0>2d"))
        world_view_client.add_joint_values(name, 
                            "/{}/generated".format(world_view_folder), 
                            joint_values)
          
    print("Now follow waypoints")
    while True:
        joint_path = JointPath(waypoints[0].joint_set, waypoints)
        await full_body_move_group.move_joints(joint_path, velocity_scaling=0.5)
        reversed_joint_path = JointPath(waypoints[0].joint_set, list(reversed(waypoints)))
        await full_body_move_group.move_joints(reversed_joint_path, velocity_scaling=0.5)

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
