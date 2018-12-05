"""
In this example, the jogging interface is used to rotate the robot around its 
torso joint. 

Thi illustrates how velocities are applied to a joint and adjusted.
"""   

import time 

from xamla_motion.data_types import JointValues, JointSet

import example_utils
from example_07.jogging_client import JoggingClient

def rotate(velocity: float, joint_name: str, jogging_client: JoggingClient) -> None:
    """ 
    Rotates around the joint defined by joint_name

    Parameters
    ----------
    velocity : float
        A value between [-1, 1], describing velocity and direction of the rotation
    joint_name : str
        The name of the joint to be rotated  
    jogging_client : JoggingClient
        The JoggingClient
    """

    # We split the velocity in its direction and the absolute value 
    abs_velocity = abs(velocity) 
    # Creating JointValues containing exclusively the current joint
    jv = JointValues(JointSet([joint_name]), [velocity] ) 
    # Send the velocity scaling and direction to jogging client
    # Scaling accepts values in range [0,1]
    jogging_client.set_velocity_scaling(abs_velocity)
    # The velocity given to send_velocities solely describes the direction of the rotation
    jogging_client.send_velocities(jv)

def main():
    torso_joint_name = example_utils.get_torso_joint_name()
    #move_group = example_utils.get_move_group()
    jogging_client = JoggingClient()
    jogging_client.set_move_group_name(example_utils.get_move_group_name())

    #Begin tracking
    jogging_client.toggle_tracking(True)
    N = 200
    frequency = 50
    for i in range(N):    
        velocity = (i - N/2) *2 /N
        if (i+1) % 10 == 0:     
            print("Call {} of {}. Velocity {} ".format(i+1, N, velocity))
        # The value of velocity go from -1 linearly to +1
        # This describes a slowing down rotation to one side, followed by 
        # accelerating rotation to the other side

        rotate(velocity=velocity, joint_name=torso_joint_name, jogging_client=jogging_client) 
        time.sleep(1/frequency)

    # Stop tracking
    jogging_client.toggle_tracking(False)

if __name__ == '__main__':
    main()

