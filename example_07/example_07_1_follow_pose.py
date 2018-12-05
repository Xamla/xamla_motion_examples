"""
In this example we repeatedly read a Pose from world view, and use the jogging 
interface to jog to that pose. The user can move the end effector around by 
updating the pose.
"""   

import time 

from xamla_motion.world_view_client import WorldViewClient

import example_utils
from example_07.jogging_client import JoggingClient

def main():
    world_view_folder = "example_07_jogging"

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    jogging_client = JoggingClient()
    jogging_client.set_move_group_name(example_utils.get_move_group_name())

    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    # Write pose to World View, so the tracking begins with the current end effector pose
    world_view_client.update_pose( "TrackingPose",  world_view_folder, current_pose)
    #Begin tracking
    jogging_client.toggle_tracking(True)

    for i in range(1000):    
        print("{} of {}".format(i, 1000))
        # Read the current Pose from WorldView
        pose = world_view_client.get_pose("TrackingPose", world_view_folder)

        jogging_client.send_set_point(pose)
        # Send set point in a frequency of 50 Hz, so that the arm keeps moving
        time.sleep(0.02)

    # Stop tracking
    jogging_client.toggle_tracking(False)

if __name__ == '__main__':
    main()

