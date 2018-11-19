""" 
This example takes several place poses and pre place joint values and a home joint value, to apply 
a pick and place operation from home to every pose
"""
from typing import List
import example_utils

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.data_types import JointValues, Pose, JointPath
from xamla_motion.motion_client import MoveGroup

import asyncio
from xamla_motion.utility import register_asyncio_shutdown_handler 

def main(poses: List[Pose], pre_place_jvs: List[JointValues], home: JointValues, move_group : MoveGroup) :
    """
    This function does a pick and place motion to every pose 

    Parameters
    ----------
    poses : List[Pose]
        Defines the place poses to be addressed
    pre_place_jvs : List[JointValues]
        Defines the joint values of the pre place positions
    home : JointValues
        The joint values, to which the robot returns after place
    move_group : MoveGroup
    """

    loop = asyncio.get_event_loop()
    # Register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    async def place(jv_pre_place: JointValues, place: Pose) -> None:
        """
        This asynchronous function lets the robot move from home to every place position and 
        back in a "pick and place" manner 
        The movement from pre place pose to place pose and back is done by moving the end 
        effector in a linear fashion

        Parameters
        ----------
        jv_pre_place : JointValues
            The pre place configuration
        jv_place : JointValues
            The place configuration

        """
        end_effector = move_group.get_end_effector() 
        # Creates a joint path over the joint values to the target pose
        joint_path = JointPath(home.joint_set, [home, jv_pre_place ])
        # Move over the joints to target pose
        await move_group.move_joints_collision_free(joint_path)
        await end_effector.move_poses_collision_free(place, 
                                                seed = jv_pre_place,
                                                velocity_scaling = 0.2)
        
        # do something, e.g. place an object 

        pre_place_pose = end_effector.compute_pose(jv_pre_place)
        await end_effector.move_poses_collision_free(pre_place_pose,
                                                    seed = jv_pre_place,
                                                    velocity_scaling = 0.2)
        # Creates a joint path over the joint values to the home pose
        joint_path_back = JointPath(home.joint_set, [jv_pre_place, home ])
        # Move back over the joint path
        await move_group.move_joints_collision_free(joint_path_back)

    try: 
        # Move to home position 
        loop.run_until_complete(move_group.move_joints_collision_free(home))
        # For every pose we want to address, do a pick and place 
        for i in range(len(pre_place_jvs)):
            print("Placing element {}".format(i))
            loop.run_until_complete(place(pre_place_jvs[i], poses[i]))
    finally:
        loop.close()

if __name__ == '__main__':
    # Called when running this script standalone
    world_view_folder = "example_02_palletizing/example_pick_place_poses"

    move_group = example_utils.get_move_group()

    world_view_client = WorldViewClient()

    # Read poses from world view 
    pose_1 = world_view_client.get_pose("Pose_1", world_view_folder)
    pose_2 = world_view_client.get_pose("Pose_2", world_view_folder)
    pose_3 = world_view_client.get_pose("Pose_3", world_view_folder)
    # Read poses from world view 
    joint_values_1 = world_view_client.get_joint_values("JointValues_1", world_view_folder)
    joint_values_2 = world_view_client.get_joint_values("JointValues_2", world_view_folder)
    joint_values_3 = world_view_client.get_joint_values("JointValues_3", world_view_folder)
    home = world_view_client.get_joint_values("Home", world_view_folder)
    poses = [pose_1, pose_2, pose_3]
    joint_values = [joint_values_1, joint_values_2, joint_values_3]
    main(poses, joint_values, home, move_group)


