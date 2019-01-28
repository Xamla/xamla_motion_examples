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

import example_utils 

def calculate_place_joint_values(poses: List[Pose], 
                                pre_place_jvs: List[JointValues], 
                                move_group: MoveGroup, 
                                world_view_client: WorldViewClient) \
                            -> List[JointValues]:
    """ 
    Calculates the place joint values

    Since we want the robot to move from pre place to place pose, we use the 
    corresponding pre place joint values as seed for the place joint values.

    Parameters
    ----------
    poses : List[Pose]
        A list of poses for which the joint values should be calculated
    pre_place_jvs : List[JointValues]
        The seeds for every pose
    move_group : MoveGroup
    world_view_client: WorldViewClient

    Returns
    ------  
    List[JointValues]
        A list of joint values for every pose
    """
    end_effector = move_group.get_end_effector()
    place_jvs = []
    for i in range(len(pre_place_jvs)):
        # The i-th pre place joint values are used for the i-th position as seed 
        joint_values = end_effector.inverse_kinematics(poses[i], 
                                                collision_check = True, 
                                                seed = pre_place_jvs[i])
        place_jvs.append(joint_values)
    return place_jvs


def main(poses: List[Pose], pre_place_jvs: List[JointValues], home: JointValues) :
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
    """

    loop = asyncio.get_event_loop()
    # Register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    place_jvs = calculate_place_joint_values(poses, 
                                        pre_place_jvs, 
                                        move_group, 
                                        world_view_client)

    async def place(jv_pre_place: JointValues, jv_place: JointValues) -> None:
        """
        This asynchronous function lets the robot move from home to every place position and 
        back in a "pick and place" manner 

        Parameters
        ----------
        jv_pre_place : JointValues
            The pre place configuration
        jv_place : JointValues
            The place configuration

        """
        # Creates a joint path over the joint values to the target pose
        joint_path = JointPath(home.joint_set, [home,jv_pre_place, jv_place ])
        # Move over the joints to target pose
        await move_group.move_joints_collision_free(joint_path)

        # do something, e.g. place an object 

        # Creates a joint path over the joint values to the home pose
        joint_path_back = JointPath(home.joint_set, [jv_place, jv_pre_place, home ])
        # Move back over the joint path
        await move_group.move_joints_collision_free(joint_path_back)

    try: 
        # Move to home position 
        loop.run_until_complete(move_group.move_joints_collision_free(home))
        # For every pose we want to address, do a pick and place 
        for i in range(len(pre_place_jvs)):
            print("Placing element {}".format(i+1))
            loop.run_until_complete(place(pre_place_jvs[i], place_jvs[i]))
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
    end_effector = example_utils.get_move_group().get_end_effector()
    poses = [pose_1, pose_2, pose_3]
    joint_values = [joint_values_1, joint_values_2, joint_values_3]
    main(poses, joint_values, home)
