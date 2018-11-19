""" 
Create boxes for a list of Poses
"""

from typing import List

from pyquaternion import Quaternion

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.data_types import Pose
from xamla_motion.data_types import CollisionObject, CollisionPrimitive

import example_utils

def main(poses: List[Pose],  size = (0.09, 0.09, 0.01)) -> CollisionObject:
    """
    Creates a bunch of boxes located relative to corresponding poses with an

    Parameters
    ----------
    poses : List[Pose]
        A list of poses where the boxes should be placed
    size : Tuple[float, float, float]
        Size of the box

    Returns
    ------  
    CollisionObject
        The grid of boxes as a CollisionObject instance
    """

    createBoxOfPose = lambda pose : CollisionPrimitive.create_box(size[0], size[1], size[2],  pose)
    return CollisionObject(list(map( createBoxOfPose , poses)))


if __name__ == '__main__':
    # Called when running this script standalone
    world_view_folder = "example_02_palletizing/example_create_collision_boxes"

    move_group = example_utils.get_move_group()

    world_view_client = WorldViewClient()

    # Read poses from world view 
    poses = world_view_client.query_poses(world_view_folder)

    boxes = main(poses, (0.2, 0.2, 0.2))
    # Save the generated collision boxes in world view
    world_view_client.add_folder("generated", world_view_folder)
    world_view_client.add_collision_object("collision_matrix", 
                                "/{}/generated".format(world_view_folder), 
                                boxes)

    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)
