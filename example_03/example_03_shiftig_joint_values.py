"""
This examples reads all the joint values in the world view and tries to 
apply a shift defined by two poses.

The first pose acts as a reference frame, while the second defines the shift relatively to the reference pose
The shift consists of translation and rotation

This is done by getting the current pose of the joint value, applying the 
translation of the shift, and then applying inverse kinematics to 
generate the joint values anew at the proposed position
"""

from typing import List 

import numpy as np

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import JointValues, Pose

from xamla_motion.xamla_motion_exceptions.exceptions import ServiceException

# This guard alows the script to be called stand alone, adding example_utils from project folder
import sys
import os
# add parent folder to sys.path, to include example utils when running alone
if "__file__" in locals():
    sys.path.append( os.path.join(os.path.dirname(__file__), '..'))
import example_utils 

def find_shifted_joint_values(joint_values: JointValues, shift: np.array, move_group: MoveGroup) -> JointValues:
    """Find joint values for the shifted pose of the joint_values end effector, if possible. 
    
    This function takes a translation vector and joint values and tries to find corresponding
    joint values shifted by "shift" in Cartesian Space

    This is done by getting the current pose of the end effector joint values, applying the 
    translation of the shift, and then applying inverse kinematics to 
    generate the joint values anew at the proposed position


    Parameters
    ----------
    jv_pre_place : JointValues
        The pre place configuration
    jv_place : JointValues
        The place configuration
    
    Exceptions
    ----------
    ServiceException
        Raised when the inverse kinematics operation was not successfull
    """
    # Get the pose of the end effetcor
    end_effector = move_group.get_end_effector()

    old_pose = end_effector.compute_pose(joint_values)
    newPose = Pose(old_pose.translation + shift, old_pose.quaternion)
    shifted_joint_values = end_effector.inverse_kinematics(newPose, seed=joint_values, collision_check = True)
    return shifted_joint_values


def main( joint_values_list: List[JointValues], shift: np.array) -> List[JointValues]:
    """
    This function takes a translation vector and a list of joint values and tries to find corresponding
    joint values shifted by "shift" in Cartesian Space
    """
    move_group = example_utils.get_move_group()

    shifted_joint_values_list = []
    for joint_values in joint_values_list:
        try:
            shifted_joint_values = find_shifted_joint_values(joint_values, shift, move_group)
            shifted_joint_values_list.append(shifted_joint_values)
            print("Could shift joint values.")
        except ServiceException as e:
            # print(e)
            print("Could not shift joint values.".format(joint_values))

    return shifted_joint_values_list
    

if __name__ == '__main__':
    # Called when running this script standalone
    world_view_folder = "example_03_shifting"

    world_view_client = WorldViewClient()





    # Read joint values from world view 
    joint_values_list = world_view_client.query_joint_values(world_view_folder)

    # The vector that defines the translation
    shift = np.array([0.0, 0.0, 0.5])
    shifted_joint_values_list = main(joint_values_list, shift)
    # Save the generated collision boxes in world view
    world_view_client.add_folder("generated", world_view_folder)
    for i in range(len(shifted_joint_values_list)):
        joint_values = shifted_joint_values_list[i]
        world_view_client.add_joint_values("joint_value_{}".format(i), 
                                    "/{}/generated".format(world_view_folder), 
                                    joint_values)
    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)
