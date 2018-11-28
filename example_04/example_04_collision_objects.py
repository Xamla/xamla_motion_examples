"""
This example generates some collision objects at poses read from world view.
Furthermore, collision free movement is show while (de)activating some of the 
obstacles.

"""
import asyncio

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import CollisionObject, CollisionPrimitive

from xamla_motion.utility import register_asyncio_shutdown_handler 

from xamla_motion.xamla_motion_exceptions.exceptions import ArgumentError

import example_utils 

def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a generated folder to world view, deletes content if existand"""
    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)

def collision_object_exists(name: str, path: str, world_view_client: WorldViewClient) -> bool:
        """ 
        Since adding elements to world view is asynchronous to our python script,
        we (busy) wait until it is there, to avoid ignoring the collision object
        """
        try:
            coll_obj = world_view_client.get_collision_object(name, path)
            return True
        except ArgumentError as e:
            return False

def main( ) :
    world_view_folder = "example_04_collision_objects"

    world_view_client = WorldViewClient()

    move_group = example_utils.get_move_group()

    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    # Read joint values for begin and end configuration
    jv_start = world_view_client.get_joint_values("Start", world_view_folder)
    jv_end = world_view_client.get_joint_values("End", world_view_folder)

    # Read poses from world view 
    poses_dict = world_view_client.query_poses("{}/poses".format(world_view_folder))

    # Read names too to associate the joint values to the poses
    poses_names = list(map( lambda posix_path: posix_path.name, list(poses_dict.keys())))
    # Create no names for the collision objects
    collision_names = list(map( lambda name: "collision_obect_of_{}".format(name), poses_names))
    
    # Get poses of dictionary
    poses = list(poses_dict.values())

    add_generated_folder(world_view_client, world_view_folder)

    # Store collision box for every positionin collision_objects
    collision_objects = {}
    for i in range(len(poses)):
        pose = poses[i]
        name = collision_names[i]
        box = CollisionPrimitive.create_box(0.15, 0.15, 0.15, pose)
        coll_obj = CollisionObject([box])
        collision_objects[name] = coll_obj

    async def move_back_and_forth(jv_start, jv_end):
        """ 
        Moves back and forth between jv_start and jv_end, evading collision objects

        
        """
        
        print("Move to start position")
        # First, move to start joint values
        await move_group.move_joints_collision_free(jv_start, velocity_scaling=0.5)

        # Now activate one collision object and observe how the robot finds a way
        # TODO: Update code when python api support activating/deactivating of collision objects
        for name in collision_names:
            print("Moving around obstacle  \"{}\".".format(name))
            # activate "name"
            coll_obj = collision_objects[name]
            # Add element to world view
            world_view_client.add_collision_object(name, 
                            "/{}/generated".format(world_view_folder), 
                            coll_obj)

            while not collision_object_exists(name, "/{}/generated".format(world_view_folder), world_view_client):
                print("Does not exist yet")

            # move to end location
            await move_group.move_joints_collision_free(jv_end, velocity_scaling=0.5)

            # Remove element from world view
            world_view_client.remove_element(name, 
                            "/{}/generated".format(world_view_folder))
            # switch start and end
            jv_temp = jv_end
            jv_end = jv_start
            jv_start = jv_temp

    try:
        loop.run_until_complete(move_back_and_forth(jv_start, jv_end))
    finally:
        loop.close()

    # clean up
    world_view_client.remove_element("generated", world_view_folder)
   
if __name__ == '__main__':
    # Called when running this script standalone
    main()
    
