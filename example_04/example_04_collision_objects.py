"""
This example generates some collision objects at poses read from world view.
Furthermore, collision free movement is show while (de)activating some of the 
obstacles.

"""
from typing import List 

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import JointValues, Pose
from xamla_motion.data_types import CollisionObject, CollisionPrimitive

import example_utils 

def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a generated folder to world view, deletes content if existand"""
    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)

def main( ) :
    world_view_folder = "example_04_collision_objects"

    world_view_client = WorldViewClient()

    move_group = example_utils.get_move_group()

    # Read joint values for begin and end configuration
    jv_start = world_view_client.get_joint_values("Start", world_view_folder)
    jv_end = world_view_client.get_joint_values("End", world_view_folder)

    # Read poses from world view 
    poses_map = world_view_client.query_poses("{}/poses".format(world_view_folder))

    # Read names too to associate the joint values to the poses
    poses_names = list(map( lambda posix_path: posix_path.name, list(poses_map.keys())))
    # Create no names for the collision objects
    collision_names = list(map( lambda name: "collision_obect_of_{}".format(name), poses_names))

    poses = list(poses_map.values())

    add_generated_folder(world_view_client, world_view_folder)

    # add a collision box for every position, stored locally in collision_objects
    collision_objects = {}
    for i in range(len(poses)):
        pose = poses[i]
        name = collision_names[i]
        box = CollisionPrimitive.create_box(0.1, 0.1, 0.2, pose)
        coll_obj = CollisionObject([box])
        collision_objects[name] = coll_obj
        world_view_client.add_collision_object(name, 
                            "/{}/generated".format(world_view_folder), 
                            coll_obj)

    # TODO: First, move to begin joint values
    
    # Now deactivate one collision object and observe how the robot finds a way
    for name in collision_names:
        # deactivate "name"
        coll_obj = collision_objects[name]
        # coll_obj.


    input("Press enter to clean up")
    world_view_client.remove_element("generated", world_view_folder)
   
if __name__ == '__main__':
    # Called when running this script standalone
    
    main()




    
