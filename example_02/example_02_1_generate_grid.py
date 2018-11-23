"""
Given a pose, generate a grid of poses
The pose describes a corner and the orientation of the grid.
"""
from typing import List

import numpy as np

from pyquaternion import Quaternion

from xamla_motion.data_types import  Pose

def main(pose: Pose, xSize: int, ySize: int, xStepSize: float , yStepSize: float) -> List[Pose]:
    """
    This function uses a pose to calculate a grid of poses

    Parameters
    ----------
    pose : Pose
        Defines the position and orientation of the grid
    xSize : int
        Number of elements of the grid in x-direction
    ySize : int
        Number of elements of the grid in y-direction
    xStepSize : float
        The distance between the poses in x-direction
    yStepSize : float
        The distance between the poses in y-direction

    Returns
    ------  
    List[Pose]
        A list of generated poses
    """

    # The point defines the rotation and is a corner of the grid
    rotation = pose.quaternion
    # Calculate the delta of each step in x- and y-direction
    # For that, we scale the unit vector pointing in x-direction/y-direction
    # and then rotate it by the rotation of the pose
    delta_x = rotation.rotate(np.array([xStepSize, 0.0, 0.0]))
    delta_y = rotation.rotate(np.array([0.0, yStepSize, 0.0]))
    poses = []  # type: List[Pose]
    for y_index in range(ySize):
        for x_index in range(ySize):
            translation = pose.translation + x_index*delta_x + y_index*delta_y
            poses.append(Pose(translation, rotation))
    return poses
    

if __name__ == '__main__':
    # Called when running this script standalone
    translation = [0.6089578, 0.3406782, 0.208865]
    rotation = Quaternion(w=0.231852, x=0.33222, y=0.746109, z=0.528387)
    pose = Pose(translation, rotation) 
    poses = main(pose, 3, 3, 0.05, 0.05)
    print(poses)

