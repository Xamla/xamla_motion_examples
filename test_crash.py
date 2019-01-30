""" 
This example takes several poses and a home joint value, to apply 
a pick and place operation from home to every pose

An alternate version of example_02_4, which calculates the joint_values for
the place poses using the pre place joint values as corresponding seeds instead
of calling end_effector.move_cartesian_linear.
The difference is that in this example we move are using 
MoveJointsCollisionFreeOperation instead of MoveCartesianLinearOperation.

"""
from typing import List

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.data_types import JointValues, Pose, JointPath
from xamla_motion.motion_client import MoveGroup

import asyncio
from xamla_motion.utility import register_asyncio_shutdown_handler 
from xamla_motion.robot_chat_client import RobotChatClient, RobotChatSteppedMotion

import example_utils 

def main(jv_prePlace, home: JointValues) :
    move_group = example_utils.get_move_group()
    loop = asyncio.get_event_loop()
    # Register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)
    try: 
        # Run to first element
        loop.run_until_complete(move_group.move_joints_collision_free(jv_prePlace))
        # This will crash too 
        # loop.run_until_complete(move_group.move_joints_collision_free(home))

        robot_chat = RobotChatClient()

        stepped_motion_client = move_group.move_joints_collision_free_supervised(home)
        robot_chat_stepped_motion = RobotChatSteppedMotion(robot_chat,
                                                    stepped_motion_client,
                                                    move_group.name)
        # This will crash. 
        loop.run_until_complete(robot_chat_stepped_motion.handle_stepwise_motions())
    finally:
        loop.close()

if __name__ == '__main__':
    # Called when running this script standalone


    world_view_client = WorldViewClient()

    jv_prePlace = world_view_client.get_joint_values("03_PrePlace","example_01_pick_place")

    home = world_view_client.get_joint_values("Home",  "example_02_palletizing/example_pick_place_poses")
    end_effector = example_utils.get_move_group().get_end_effector()
    main(jv_prePlace, home)
