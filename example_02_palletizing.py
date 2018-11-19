"""
This example shows how the robot can pick and place for a grid of poses


First, a grid of place and pre place poses is calculated out of a single 
pose in world view, which describes an edge and the orientation of the grid.
The manipulation of this pose results in a different grid, so one can see if 
and which solution is found for the operation.

Joint values are calculated by using inverse  kinematics for the place and 
pre place poses. These robot configuration are saved in world view.

Then, the robot applies a pick and place for all the poses of the grid. 
"""
from typing import List, Tuple

import numpy as np

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import JointValues, Pose, CartesianPath, JointPath
from xamla_motion.data_types import CollisionObject, CollisionPrimitive

import asyncio
from xamla_motion.utility import register_asyncio_shutdown_handler 

import example_utils 
import example_generate_grid
import example_create_collision_boxes
import example_create_joint_values_from_poses

def generate_folders(world_view_client: WorldViewClient) -> None:
    """ 
    Generate the folder we want to use in world view
    """

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", "/example_02_palletizing")
    except Exception as e:
        None
    # Add a folder to hold calculated joint values to be accessed in  world view
    world_view_client.add_folder("generated", "/example_02_palletizing")
    world_view_client.add_folder("collision_objects", "/example_02_palletizing/generated")
    # To show all generated joint values in the world view
    world_view_client.add_folder("pre_place_joint_values", "/example_02_palletizing/generated")
    world_view_client.add_folder("place_joint_values", "/example_02_palletizing/generated")
    

def calculate_pre_place_joint_values(pre_place_poses: List[Pose], 
                                    jv_home: JointValues, 
                                    move_group: MoveGroup, 
                                    world_view_client: WorldViewClient) \
                                -> List[JointValues]:
    """ 
    Calculates the pre place joint values
    
    Since we want the robot arm to go back and forth from the home configuration 
    to the poses on the grid, we use the home joint value as a const seed for 
    every pose to minimize the distance for the joints to make.
    Calling  inverse_kinematics_many with "const_seed = True" lets us exclusively
    using the jv_home joint values as seed. 
    "const_seed = False" would use the  previously calculated joint values as seed 
    for the next on when traversing pre_place_poses. This could be useful when  
    we went from pose to pose without visiting the jv_home configuration in between.

    Parameters
    ----------
    pre_place_poses : List[Pose]
        A list of poses for which the joint values should be calculated
    jv_home : JointValues
        The seed to be used
    move_group : MoveGroup
    world_view_client: WorldViewClient

    Returns
    ------  
    List[JointValues]
        A list of joint values for every pose
    """
    end_effector = move_group.get_end_effector()
    try:
        pre_place_jvs = example_create_joint_values_from_poses.main(pre_place_poses,
                                                    jv_home,
                                                    end_effector)
    except Exception as e:
        print("The inverse kinematics operation could not be applied on all the positions.")
        world_view_client.remove_element("generated", "/example_02_palletizing")
        raise e
    # export every calculated joint values to world view to illustrate the result
    for i in range(len(pre_place_jvs)):
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                                "/example_02_palletizing/generated/pre_place_joint_values", 
                                pre_place_jvs[i])
    return pre_place_jvs

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
        # export every calculated joint values to world view to illustrate the result
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                                "/example_02_palletizing/generated/place_joint_values", 
                                place_jvs[i])
    return place_jvs

def main(xSize: int, ySize: int, xStepSize: float , yStepSize: float):
    """
    The parameter for this function describes the size of the grid.
    To manipulate orientation and position, one can alter the Pose named "GridPose" 
    in the folder "example_02_palletizing" of the world view.

    Parameters
    ----------
    xSize : int
        Number of elements of the grid in x-direction
    ySize : int
        Number of elements of the grid in y-direction
    xStepSize : float
        The distance between the poses in x-direction
    yStepSize : float
        The distance between the poses in y-direction
    """

    # Create move group instance targeting the right arm of the robot
    move_group = example_utils.get_move_group()

    # Get the gripper attached at the right arm
    wsg_gripper = example_utils.get_gripper(move_group)

    # Create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()

    # Generate the folders used by this example in world vew
    generate_folders(world_view_client)

    # Get the pose of the position which defines the location and rotation of the grid
    pose = world_view_client.get_pose("GridPose","example_02_palletizing")
    jv_home = world_view_client.get_joint_values("Home","example_02_palletizing")

    loop = asyncio.get_event_loop()
    # Register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    # Get the target place poses
    poses = example_generate_grid.main(pose, xSize, ySize, xStepSize, yStepSize)
    rotation = pose.quaternion

    # Calculate the orthogonal vector of the plane we want to span
    # Since we use [1,0,0] and [0,1,0] vectors to span the grid relatively to the 
    # pose orientation, [0,0,1] is orthogonal to the grid
    orthogonal = rotation.rotate(np.array([0,0,1]))

    # For visualization and possible collisions, add some boxes below the positions 
    # we want to visit
    getBoxPose = lambda pose : Pose(pose.translation + (orthogonal * (0.12)), pose.quaternion) 
    boxPoses = list(map(getBoxPose, poses))
    boxes = example_create_collision_boxes.main(boxPoses, (xStepSize*0.9, yStepSize*0.9, 0.01))
    world_view_client.add_collision_object("collision_matrix", 
                                    "/example_02_palletizing/generated/collision_objects", 
                                    boxes)

    # Now calculate the pre place poses, which hover over the desired place poses
    # Since we want to access every pose in the grid "from above", we apply a 
    # translation orthogonal to the place poses for the pre place poses  
    #func = lambda pose : Pose(pose.translation + (orthogonal * (-0.1)), pose.quaternion) 
    func = lambda pose : Pose(pose.translation + (pose.quaternion.rotate(np.array([0,0,1])) * (-0.1)), pose.quaternion) 
    pre_place_poses = list(map( func , poses))

    # Now that we have all the poses we want to visit, we should find the 
    # corresponding joint values
    pre_place_jvs = calculate_pre_place_joint_values(pre_place_poses, 
                                                jv_home, 
                                                move_group, 
                                                world_view_client) 
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
        joint_path = JointPath(jv_home.joint_set, [jv_home,jv_pre_place, jv_place ])
        # Move over the joints to target pose
        await move_group.move_joints_collision_free(joint_path)

        # do something, e.g. place an object 

        # Creates a joint path over the joint values to the home pose
        joint_path_back = JointPath(jv_home.joint_set, [jv_place, jv_pre_place, jv_home ])
        # Move back over the joint path
        await move_group.move_joints_collision_free(joint_path_back)

    try: 
        # Move to home position 
        loop.run_until_complete(move_group.move_joints_collision_free(jv_home))
        # For every pose we want to address, do a pick and place 
        for i in range(len(pre_place_jvs)):
            print("Placing element {}".format(i))
            loop.run_until_complete(place(pre_place_jvs[i], place_jvs[i]))
    finally:
        loop.close()

    world_view_client.remove_element("generated", "/example_02_palletizing")

if __name__ == '__main__':
    main(3, 3, 0.1, 0.1)