
from typing import List

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.data_types import JointValues, Pose, JointPath
from xamla_motion.motion_client import MoveGroup

import asyncio
from xamla_motion.utility import register_asyncio_shutdown_handler 
from xamla_motion.robot_chat_client import RobotChatClient, RobotChatSteppedMotion

def main(jv_prePlace, home: JointValues) :
    move_group = MoveGroup("/sda10f/right_arm_torso")

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
    main(jv_prePlace, home)
