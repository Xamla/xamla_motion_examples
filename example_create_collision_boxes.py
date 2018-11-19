""" 
Create boxes for a list of Poses
"""
from typing import List

from xamla_motion.data_types import Pose
from xamla_motion.data_types import CollisionObject, CollisionPrimitive

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
    translation = [0.6089578, 0.3406782, 0.208865]
    rotation = Quaternion(w=0.231852, x=0.33222, y=0.746109, z=0.528387)
    pose = Pose(translation, rotation) 
    collision_object = main([pose], (0.2, 0.2, 0.2))
    

