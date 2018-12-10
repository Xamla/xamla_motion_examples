"""
In this example we repeatedly read a Pose from world view, and use the jogging 
interface to jog to that pose. The user can move the end effector around by 
updating the pose.
"""   

import time 

from xamla_motion.world_view_client import WorldViewClient

import example_utils
from example_07.jogging_client import JoggingClient
from example_07.example_07_jogging_feedback import callback_function as feedback_function

def track_point(time_amount: float, 
                frequency: float,  
                point_name: str,
                get_pose,
                jogging_client: JoggingClient) -> None:
    """ 
    Tracks a point 

    Parameters
    ----------
    time_amount : float
        Amount of time the tracking should occur
    frequency : float
        Frequency of the calls to JoggingClient in Hz
    get_pose : lambda function
        Function which returns a Pose when called
    jogging_client : JoggingClient
        The JoggingClient
    """

    # Calculate the number of calls to jogging client based on frequency and time
    N = int(time_amount*frequency)
    print(N)
    for i in range(N):  
        if (i+1) % 10 == 0:     
            print("Call {} of {}".format(i+1, N))
        # Read the current Pose from WorldView
        pose = get_pose()
        jogging_client.send_set_point(pose)
        # Send set point in a frequency of 50 Hz, so that the arm keeps moving
        time.sleep(1/frequency)

def main():
    world_view_folder = "example_07_jogging"

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    jogging_client = JoggingClient()
    jogging_client.set_move_group_name(example_utils.get_move_group_name())
    # register feedback function, to get alerted when an error occurs
    jogging_client.register(feedback_function)

    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    point_name = "TrackingPose"
    # Write pose to World View, so the tracking begins with the current end effector pose
    world_view_client.update_pose( point_name,  world_view_folder, current_pose)

    get_pose = lambda : world_view_client.get_pose(point_name, world_view_folder)

    #Begin tracking
    jogging_client.toggle_tracking(True)
    # Track for 20 seconds with a frequency of 50 Hz
    track_point(time_amount=20, 
                frequency=50, 
                point_name=point_name, 
                get_pose=get_pose,
                jogging_client=jogging_client)

    # Stop tracking
    jogging_client.toggle_tracking(False)
    # Unregister the feedback function
    jogging_client.unregister(feedback_function)
    
if __name__ == '__main__':
    main()
