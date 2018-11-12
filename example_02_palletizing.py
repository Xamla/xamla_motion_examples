from typing import List

import asyncio

import numpy as np

from pyquaternion import Quaternion

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import JointValues, Pose
from xamla_motion.robot_chat_client import RobotChatClient, RobotChatSteppedMotion

import example_utils 



def main(number: int, width: float, offset: float):
    # create move group instance targeting the right arm of the robot
    move_group = example_utils.get_move_group()

    # get the gripper attached at the right arm
    wsg_gripper = example_utils.get_gripper(move_group)

    # get a client to communicate with the robot
    robot_chat = RobotChatClient()

    # create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()

    # get the pose of the position both palettes are stacked upon 
    pose_source = world_view_client.get_pose("source","example_02_palletizing")
    pose_target = world_view_client.get_pose("target","example_02_palletizing")

    # assumes 0.1 x 0.1 m tiles, which collision boxes are constructed from the parameters given to the main function. 
    # all palletes are assumed to be at pose source at first  
    # add a new folder to store the new collision boxes
    world_view_client.add_folder("collision_objects", "/example_02_palletizing")
    
    
    poses_target = 
    def get_sources_poses(source: Pose) -> List[Pose]: 
        poses_sources = []
        return poses_sources

    poses_sources = get_sources_poses(pose_source)

    

if __name__ == '__main__':
    numberOfPalletes = 4
    widthOfPalletes = 0.02
    offsetOfPalletes = 0.10
    main(numberOfPalletes, widthOfPalletes, offsetOfPalletes)