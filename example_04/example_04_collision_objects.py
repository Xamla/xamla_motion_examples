"""
This example generates some collision objects at poses read from world view.
Furthermore, collision free movement is show while (de)activating some of the 
obstacles.

"""
from typing import List 

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import JointValues, Pose

# This guard alows the script to be called stand alone, adding example_utils from project folder
import sys
import os
# add parent folder to sys.path, to include example utils when running alone
if "__file__" in locals():
    sys.path.append( os.path.join(os.path.dirname(__file__), '..'))
import example_utils 


def main( ) :
    world_view_folder = "example_04_collision_objects"

    world_view_client = WorldViewClient()

    # Read poses from world view 
    poses = world_view_client.query_poses("{}/poses".format(world_view_folder))


    # Get objects for every pose, deactivate them


    move_group = example_utils.get_move_group()


    world_view_client.add_folder("generated", world_view_folder)

    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)
   
if __name__ == '__main__':
    # Called when running this script standalone
    
    main()




    
