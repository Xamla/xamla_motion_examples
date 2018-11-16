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
import signal
import functools



def get_plane_from_points(point1, point2, point3):
    """Returns a plane definition given 3 points  """
    p1, p2, p3 = point1.translation, point2.translation, point3.translation
    v1 = p3 - p1
    v2 = p2 - p1
    a, b, c = np.cross(v1, v2)
    d = - np.dot((a,b,c), p1)
    return a, b, c, d


def get_grid_poses(point, size: Tuple[int, int], step: Tuple[float, float]):
    """This function uses a points to calculate a grid of poses

    The rotation of the poses are set by point
    """

    # The point defines the rotation and the begining of the grid
    rotation = point.quaternion

    # Define a vector in x and y direction
    delta_x = rotation.rotate(np.array([step[0], 0.0, 0.0]))
    delta_y = rotation.rotate(np.array([0.0, step[1], 0.0]))

    print(type(delta_x))

    # Assuming that the first position has the correct orientation 
    poses = []  # type: List[Pose]
    for y_index in range(size[1]):
        for x_index in range(size[0]):
            translation = point.translation + x_index*delta_x + y_index*delta_y
            pose = Pose(translation, rotation)
            poses.append(pose)
    return poses
    

def create_collision_boxes(poses: List[Pose], vector: np.array) -> CollisionObject:
    """Creates a bunch of boxes located relative to corresponding poses with an offset defined by vector """
    func = lambda pose : CollisionPrimitive.create_box(0.09, 0.09, 0.01, Pose(pose.translation + vector, pose.quaternion))
    return list(map( func , poses))


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

    # Describe in a matrix of boolean the already visited elements

    # get the pose of the position which defines the grid
    point1 = world_view_client.get_pose("Pose_1","example_02_palletizing")
    #point2 = world_view_client.get_pose("Pose_2","example_02_palletizing"):
    #point3 = world_view_client.get_pose("Pose_3","example_02_palletizing")

    jv_home = world_view_client.get_joint_values("Home","example_02_palletizing")

    end_effector = move_group.get_end_effector()

    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    poses = get_grid_poses(point1, (xSize, ySize), (xStepSize, yStepSize))

    print(poses)

    rotation = point1.quaternion
   
    orthogonal = rotation.rotate(np.array([0,0,1]))


    # For visualization and possible collisions, add some boxes below the positions we want to visit
    boxes = create_collision_boxes(poses , (orthogonal * (0.2)))
    placed_objects = [] 

    # Now calculate the poses which hover over the desired poses 
    func = lambda pose : Pose(pose.translation + (orthogonal * (-0.1)), pose.quaternion) 
    pre_place_poses = list(map( func , poses))

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", "/example_02_palletizing")
    except Exception as e:
        print("No generated folder")
    finally:
        # Add a folder to hold calculated joint values to be accessed in  world view
        world_view_client.add_folder("generated", "/example_02_palletizing")
        world_view_client.add_folder("collision_objects", "/example_02_palletizing/generated")
        #plane = CollisionPrimitive.create_plane(rotation.real, rotation.vector[0], rotation.vector[1], rotation.vector[2], point1)
        world_view_client.add_collision_object("collision_matrix", "/example_02_palletizing/generated/collision_objects", CollisionObject(boxes))
        # world_view_client.add_collision_object("collision_placed_objects", "/example_02_palletizing/generated/collision_objects", CollisionObject([plane]))



    # Now that we have all the poses we want to visit, we should find the corresponding joint values

    # First, we calculate the joint values for the pre place poses
    # Since we want the robot arm to go back and forth from the home configuration to the poses on the grid, 
    # we use the home joint values as a const seed for every pose to minimize the distance for the joints to make
    # const_seed = True means we always use the jv_home joint values as seed, and do not use the previous calculated joint value as seed
    # const_seed = False would make sense if we went from pose to pose without visiting the jv_home configuration in between
    ik_results = end_effector.inverse_kinematics_many(CartesianPath(pre_place_poses), collision_check = True, seed = jv_home, const_seed = True)
    # The following asserts that a configuration could have been found for every pose
    assert ik_results.succeeded
    # joint_path now contains the result of the ik-operation
    pre_place_jvs = ik_results.path  # type: List[] 

    # We do the same for the place positions. 
    # Since we want the robot to move from pre place to place, 
    # we use every pre place joint values as seed for the place joint values
    place_jvs = []
    for i in range(len(pre_place_jvs)):
        ik_results = end_effector.inverse_kinematics(poses[i], collision_check = True, seed = pre_place_jvs[i])
        place_jvs.append(ik_results)
   

    # To show all the calculated joint values in the world view, we add them 
    world_view_client.add_folder("pre_place_joint_values", "/example_02_palletizing/generated")
    world_view_client.add_folder("place_joint_values", "/example_02_palletizing/generated")

    for i in range(len(pre_place_jvs)):
        # export every calculated joint values to world view 
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                                "/example_02_palletizing/generated/pre_place_joint_values", 
                                pre_place_jvs[i])
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                                "/example_02_palletizing/generated/place_joint_values", 
                                place_jvs[i])




    #def place_object_at_pose(pose: Pose):
    #    """This function just adds a collision object at the pose """
    #    placed_objects.append(CollisionPrimitive.create_box(0.02, 0.02, 0.04, Pose(pose.translation + (orthogonal * (0.1)), pose.quaternion) ))
    #    world_view_client.update_collision_object("collision_placed_objects", "/example_02_palletizing/generated/collision_objects", CollisionObject(placed_objects))  

    async def place(jv_pre_place: JointValues, jv_place: JointValues):
        """Moves from home to place position and back """
        # Creates a joint path over the joint values to the target pose
        joint_path = JointPath(jv_home.joint_set, [jv_home,jv_pre_place, jv_place,  ])
        # Move over the joints to target pose
        await move_group.move_joints_collision_free(joint_path)

        # do something

        # Creates a joint path over the joint values to the home pose
        joint_path_back = JointPath(jv_home.joint_set, [jv_place, jv_pre_place, jv_home ])
        # Move back over the joint path
        await move_group.move_joints_collision_free(joint_path_back)

    try: 
        # Move to home position 
        loop.run_until_complete(move_group.move_joints_collision_free(jv_home))
        # For every pose we want to address, do a pick and place of an object
        for i in range(len(poses)):
            print("Placing element {}".format(i))
            loop.run_until_complete(place(pre_place_jvs[i], place_jvs[i]))
    finally:
        loop.close()

    input('Press enter to clean up')
    world_view_client.remove_element("generated", "/example_02_palletizing")

if __name__ == '__main__':
    main(3, 3, 0.1, 0.1)