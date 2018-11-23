""" 
In this example, joint values are calculated from poses and a seed

The resulting joint values are stored in a "generated" sub folder 
when running this script via terminal.
"""

from typing import List


from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.data_types import JointValues, Pose, CartesianPath
from xamla_motion.motion_client import EndEffector

# This guard alows the script to be called stand alone, adding example_utils from project folder
import sys
import os
# add parent folder to sys.path, to include example utils when running alone
if "__file__" in locals():
    sys.path.append( os.path.join(os.path.dirname(__file__), '..'))
import example_utils 

def main(poses: List[Pose], 
        seed: JointValues, 
        end_effector: EndEffector) \
    -> List[JointValues]:
    """ 
    Calculates joint values from poses and a seed
    
    Calling  inverse_kinematics_many with "const_seed = True" lets us 
    exclusively using the "seed" joint values as seed. 

    Parameters
    ----------
    poses : List[Pose]
        A list of poses for which the joint values should be calculated
    seed : JointValues
        The seed to be used
    end_effector: EndEffector

    Returns
    ------  
    List[JointValues]
        A list of joint values for every pose
    
    Exceptions
    ------
    Exception
        Raised when there was one or more poses for which the inverse kinematics 
        operation failed
    """
    
    cartesian_path = CartesianPath(poses)
    ik_results = end_effector.inverse_kinematics_many(cartesian_path, 
                                                    collision_check = True, 
                                                    seed = seed, 
                                                    const_seed = True)
    # Check if a configuration has been found for every pose
    if not ik_results.succeeded:
        raise Exception("The inverse kinematics operation could not be applied on all the positions.")
    # joint_path now contains the result of the ik-operation
    return ik_results.path  

if __name__ == '__main__':
    # Called when running this script standalone
    world_view_folder = "example_02_palletizing/example_create_jv_from_poses"

    move_group = example_utils.get_move_group()
    end_effector = example_utils.get_move_group().get_end_effector()

    world_view_client = WorldViewClient()
    seed = world_view_client.get_joint_values("Seed", world_view_folder)

    # Read poses from world view 
    poses = world_view_client.query_poses(world_view_folder)
    joint_values = main(poses, seed, end_effector)
    # Save the generated joint value in world view
    world_view_client.add_folder("generated", world_view_folder)
    for i in range(len(joint_values)):
        world_view_client.add_joint_values("joint_values_{}".format(str(i).zfill(2)), 
                            "{}/generated".format(world_view_folder), 
                            joint_values[i])
    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)
