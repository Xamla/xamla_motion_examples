"""
This example shows how to apply a twist to the current end_effector using the 
jogging interface 
"""

import time 

from xamla_motion.world_view_client import WorldViewClient

import example_utils
from example_07.jogging_client import JoggingClient, Twist
from example_07.example_07_jogging_feedback import callback_function as feedback_function

def apply_twist(time_amount: float, 
                frequency: float,  
                twist: Twist,
                jogging_client: JoggingClient) -> None:
    """ 
    Applies a twist for an amount of time

    Parameters
    ----------
    time_amount : float
        Amount of time the tracking should occur
    frequency : float
        Frequency of the calls to JoggingClient in Hz
    twist : Twist
        The twist to be applied
    jogging_client : JoggingClient
        The JoggingClient
    """

    # Calculate the number of calls to jogging client based on frequency and time
    N = int(time_amount*frequency)
    for i in range(N):
        if (i+1) % 10 == 0:    
            print("Call {} of {}".format(i+1, N))
        jogging_client.send_twist(twist)
        # Send set point in a frequency of 50 Hz, so that the arm keeps moving
        time.sleep(1/frequency)

def main():
    world_view_folder = ""
    jogging_client = JoggingClient()

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    move_group_name = "/sda10f/right_arm_torso"
    jogging_client.set_move_group_name(move_group_name)

    # register feedback function, to get alerted when an error occurs
    jogging_client.register(feedback_function)

    #Begin tracking
    jogging_client.toggle_tracking(True)

    # Go back and forth in x direction (linear velocity)
    forth_trans_twist = Twist(linear=[0.5,0,0], angular=[0,0,0])
    back_trans_twist = Twist(linear=[-0.5,0,0], angular=[0,0,0])
    time_amount = 2
    frequency = 50
    print("Go forth along x-axis")
    apply_twist(time_amount, frequency, forth_trans_twist, jogging_client)
    print("Go back along x-axis")
    apply_twist(time_amount, frequency, back_trans_twist, jogging_client)

    # Keep rolling
    forth_roll_twist = Twist([0,0,0], [10,0,0])
    back_roll_twist = Twist([0,0,0], [-10,0,0])
    print("Rotate forth along y-axis")
    apply_twist(time_amount, frequency, forth_roll_twist, jogging_client)
    print("Rotate back along y-axis")    
    apply_twist(time_amount, frequency, back_roll_twist, jogging_client)

    # Stop tracking
    jogging_client.toggle_tracking(False)
    # Unregister the feedback function
    jogging_client.unregister(feedback_function)

if __name__ == '__main__':
    main()

