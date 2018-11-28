"""
This examples reads all the joint values in the world view and tries to 
apply a shift defined by two poses.

The first pose acts as a reference frame, while the second defines the shift 
relatively to the reference pose. The shift consists of translation and rotation.

This is done by getting the current pose of the joint value, applying the 
translation and rotation of the shift, and then applying inverse kinematics to 
generate the joint values anew at the proposed poses.

When there are no corresponding joint values found, the pose is skipped.
"""
from typing import List 

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

def find_shifted_joint_values(joint_values: JointValues, diff_pose: Pose, 
                              move_group: MoveGroup) -> JointValues:
    """
    Find joint values for the shifted pose of the joint_values end effector, if 
    possible. 
    
    This function takes a translation vector and joint values and tries to find 
    corresponding joint values shifted by "diff_pose" in Cartesian Space

    This is done by getting the current pose of the end effector joint values, 
    applying the translation and rotation of "diff_pose", and then applying 
    inverse kinematics to generate the joint values anew at the proposed poses.

    Parameters
    ----------
    joint_values: JointValues
        The joint values to be shifted 
    diff_pose: Pose
        The pose which defines translation an rotation
    move_group: MoveGroup

    Returns
    ----------
    JointValues
        The shifted jointValues
    
    Exceptions
    ----------
    ServiceException
        Raised when the inverse kinematics operation was not successfull
    """

    # Get the pose of the end effetcor
    end_effector = move_group.get_end_effector()

    old_pose = end_effector.compute_pose(joint_values)
    # translate and rotate 
    newPose = old_pose.translate(diff_pose.translation).rotate(diff_pose.quaternion)
    # calculate joint values at new pose, which might throw an exception if not 
    # possible
    shifted_joint_values = end_effector.inverse_kinematics(newPose, 
                                    seed=joint_values, collision_check = True)
    return shifted_joint_values


def main( joint_values_list: List[JointValues], diff_pose: Pose) -> List[JointValues]:
    """
    This function takes a translation vector and a list of joint values and tries 
    to find corresponding joint values shifted by "diff_pose" in Cartesian Space

    Parameters
    ----------
    joint_values_list: List[JointValues]
        A list of joint values to be shifted 
    diff_pose: Pose
        The pose which defines translation an rotation

    Returns
    ----------
    List[JointValues]
        A list shifted JointValues for every joint value in joint_values_list 
        that could be shifted

    """
    move_group = example_utils.get_move_group()
    print("Shift joint values.")
    shifted_jvs_list = []
    for joint_values in joint_values_list:
        try:
            shifted_jvs = find_shifted_joint_values(joint_values, diff_pose, move_group)
            shifted_jvs_list.append(shifted_jvs)
            print("Success.")
        except ServiceException as e:
            # print(e)
            print("Could not shift joint values.".format(joint_values))
            # append None so we can test if this failed or not
            shifted_jvs_list.append(None)

    return shifted_jvs_list
   
if __name__ == '__main__':
    # Called when running this script standalone
    world_view_folder = "example_03_shifting"

    world_view_client = WorldViewClient()

    # Read joint values from world view 
    joint_values_list_map = world_view_client.query_joint_values(
                              "{}/jointValues".format(world_view_folder))
    joint_values_list = list(joint_values_list_map.values())
    # Read names too to associate the joint values to the poses
    names = list(map( lambda posix_path: posix_path.name, list(joint_values_list_map.keys())))
    # Create no names for joint values
    new_names = list(map( lambda name: "shifted_joint_values_of_{}".format(name), names))


    # Read poses from world view 
    reference_pose = world_view_client.get_pose("Reference", 
                              "/{}/poses".format(world_view_folder))
    shift_pose = world_view_client.get_pose("Shift", 
                              "/{}/poses".format(world_view_folder))

    # Create a pose defining the shift from the reference_poses view 
    # First undo translation and rotation of reference pose
    # then translate and rotate by the shift pose
    diff_pose = Pose( -reference_pose.translation, reference_pose.quaternion.inverse)\
      .translate(shift_pose.translation).rotate(shift_pose.quaternion)
    
    shifted_joint_values_list = main(joint_values_list, diff_pose)
    # Save the shifted joint values to world view
    print("Save the shifted joint values to world view")
    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)
    for i in range(len(shifted_joint_values_list)):

        joint_values = shifted_joint_values_list[i]
        # Test if not "None"
        if joint_values:
            name = new_names[i]
            world_view_client.add_joint_values(name, 
                                        "/{}/generated".format(world_view_folder), 
                                        joint_values)
    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)
    