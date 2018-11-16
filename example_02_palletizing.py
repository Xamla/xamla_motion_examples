"""


This example shows 




"""

from typing import List, Tuple

import asyncio

import numpy as np

from pyquaternion import Quaternion

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup, EndEffector
from xamla_motion.data_types import JointValues, Pose, CartesianPath, JointPath
from xamla_motion.data_types import CollisionObject, CollisionPrimitive
from xamla_motion.robot_chat_client import RobotChatClient, RobotChatSteppedMotion

from xamla_motion.utility import register_asyncio_shutdown_handler 

import example_utils 

# only for test


def generate_folders(world_view_client: WorldViewClient) -> None:
    # generate the folder we want to use
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

def get_grid_poses(pose: Pose, size: Tuple[int, int], step: Tuple[float, float]) -> List[Pose]:
    """
    This function uses a pose to calculate a grid of poses

        Parameters
        ----------
        pose : Pose
            Defines the position and orientation of the grid
        size : Tuple[int, int]
            Number of elements of the grid in x- and y-direction
        step : Tuple[float, float]
            The distance between the poses in x- and y-direction

        Returns
        ------  
        List[Pose]
            A list of generated poses

    """

    # The point defines the rotation and the begining of the grid
    rotation = pose.quaternion
    # Calculate the delta of each step in x- and y-direction
    # For that, we scale the unit vector pointing in x-direction/y-direction
    # and then rotate it by the rotation of the pose
    delta_x = rotation.rotate(np.array([step[0], 0.0, 0.0]))
    delta_y = rotation.rotate(np.array([0.0, step[1], 0.0]))
    poses = []  # type: List[Pose]
    for y_index in range(size[1]):
        for x_index in range(size[0]):
            translation = pose.translation + x_index*delta_x + y_index*delta_y
            poses.append(Pose(translation, rotation))
    return poses
    
def create_collision_boxes(poses: List[Pose], vector: np.array, size = (0.09, 0.09)) -> CollisionObject:
    """
    Creates a bunch of boxes located relative to corresponding poses with an offset defined by vector 

    This is just a visualization aid to locate the poses
    
    """
    func = lambda pose : CollisionPrimitive.create_box(size[0], size[1], 0.01, Pose(pose.translation + vector, pose.quaternion))
    return list(map( func , poses))


def calculate_pre_place_joint_values(pre_place_poses: List[Pose], jv_home: JointValues, move_group: MoveGroup, world_view_client: WorldViewClient) -> List[JointValues]:
    """ 
    Calculates the pre place joint values
    """
    # Now that we have all the poses we want to visit, we should find the corresponding joint values
    
    # First, we calculate the joint values for the pre place poses
    # Since we want the robot arm to go back and forth from the home configuration to the poses on the grid, 
    # we use the home joint values as a const seed for every pose to minimize the distance for the joints to make
    # const_seed = True means we always use the jv_home joint values as seed, and do not use the previous calculated joint value as seed
    # const_seed = False would make sense if we went from pose to pose without visiting the jv_home configuration in between
    end_effector = move_group.get_end_effector()
    ik_results = end_effector.inverse_kinematics_many(CartesianPath(pre_place_poses), collision_check = True, seed = jv_home, const_seed = True)
    # The following asserts that a configuration could have been found for every pose
    assert ik_results.succeeded
    # joint_path now contains the result of the ik-operation
    pre_place_jvs = ik_results.path  # type: List[] 
    for i in range(len(pre_place_jvs)):
        # export every calculated joint values to world view 
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                                "/example_02_palletizing/generated/pre_place_joint_values", 
                                pre_place_jvs[i])
    return pre_place_jvs


def calculate_place_joint_values(poses: List[Pose], pre_place_jvs: List[JointValues], move_group: MoveGroup, world_view_client: WorldViewClient) -> List[JointValues]:
    """ 
    Calculates the pre place joint values
    """
    # Now that we have all the poses we want to visit, we should find the corresponding joint values
    
    # First, we calculate the joint values for the pre place poses
    # Since we want the robot arm to go back and forth from the home configuration to the poses on the grid, 
    # we use the home joint values as a const seed for every pose to minimize the distance for the joints to make
    # const_seed = True means we always use the jv_home joint values as seed, and do not use the previous calculated joint value as seed
    # const_seed = False would make sense if we went from pose to pose without visiting the jv_home configuration in between
    end_effector = move_group.get_end_effector()
    # We do the same for the place positions. 
    # Since we want the robot to move from pre place to place, 
    # we use every pre place joint values as seed for the place joint values
    place_jvs = []
    for i in range(len(pre_place_jvs)):
        ik_results = end_effector.inverse_kinematics(poses[i], collision_check = True, seed = pre_place_jvs[i])
        place_jvs.append(ik_results)
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                                "/example_02_palletizing/generated/place_joint_values", 
                                place_jvs[i])
    return place_jvs



def main(xSize: int, ySize: int, xStepSize: float , yStepSize: float):
    """
    
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

    # create move group instance targeting the right arm of the robot
    move_group = example_utils.get_move_group()

    # get the gripper attached at the right arm
    wsg_gripper = example_utils.get_gripper(move_group)

    # get a client to communicate with the robot
    robot_chat = RobotChatClient()

    # create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()


    generate_folders(world_view_client)

    

    # get the pose of the position which defines the location and rotation of the grid
    pose = world_view_client.get_pose("Pose_1","example_02_palletizing")
    jv_home = world_view_client.get_joint_values("Home","example_02_palletizing")


    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    poses = get_grid_poses(pose, (xSize, ySize), (xStepSize, yStepSize))

    rotation = pose.quaternion
    # Calculate the orthogonal vector of the plane we want to span
    orthogonal = rotation.rotate(np.array([0,0,1]))

    # For visualization and possible collisions, add some boxes below the positions we want to visit
    boxes = create_collision_boxes(poses , (orthogonal * (0.12)), (xStepSize*0.9, yStepSize*0.9))
    world_view_client.add_collision_object("collision_matrix", "/example_02_palletizing/generated/collision_objects", CollisionObject(boxes))

    # Now calculate the pre place poses, which hover over the desired place poses
    # Since we want to access every pose in the grid "from above", we apply a 
    # translation orthogonal to the place poses for the pre place poses  
    func = lambda pose : Pose(pose.translation + (orthogonal * (-0.1)), pose.quaternion) 
    pre_place_poses = list(map( func , poses))

    pre_place_jvs = calculate_pre_place_joint_values(pre_place_poses, jv_home, move_group, world_view_client) 
    place_jvs = calculate_place_joint_values(poses, pre_place_jvs, move_group, world_view_client)

    async def place(jv_pre_place: JointValues, jv_place: JointValues):
        """Moves from home to place position and back """
        # Creates a joint path over the joint values to the target pose
        joint_path = JointPath(jv_home.joint_set, [jv_home,jv_pre_place, jv_place,  ])
        # Move over the joints to target pose
        await move_group.move_joints_collision_free(joint_path)

        # do something, e.g. grab 

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

    input('Press enter to clean up')
    world_view_client.remove_element("generated", "/example_02_palletizing")

if __name__ == '__main__':
    main(3, 3, 0.1, 0.1)