import numpy as np
import math

import asyncio
import signal
import functools

from xamla_motion.data_types import CartesianPath
from xamla_motion.motion_client import MoveGroup

from xamla_motion.utility import register_asyncio_shutdown_handler 

import example_utils

def add_sin_to_trajectory(cartesian_path: CartesianPath,
                          amplitude: float,
                          frequency: float,
                          axis: int) -> CartesianPath:
    """ 
    Applies a translation to every pose in the cartesian path in form of a sinus shape

    Parameters
    ----------
    cartesian_path : CartesianPath 
        The CartesianPath object to be altered
    amplitude : float
        The amplitude of the sinus function in mm
        (the maximal deviationfrom the pose) 
    frequency : float
        The frequency of the sinus function 
        (how long a period takes relatively to the path length)
    axis: int
        The axis on which the translation should be applied
    Returns
    ------  
    CartesianPath
        The cartesian_path altered by a sinus shaped translation
    """
        
    if axis < 0 or axis > 2:
        msg = 'axis can be x=0, y=1, z=2 but has value: {}'.format(axis)
        raise ValueError(msg)

    def sin_generator(number_of_samples: int, amplitude: float, frequency: float):
        """
        This generator calculates values for an index according to a sinus 
        function defined by the parameters.
        It yields the current index and the corresponding sinus value
            
        Parameters
        ----------
        number_of_samples : int 
            The number of samples
        amplitude : float
            The amplitude of the sinus function in mm
        frequency : float
            The frequency of the sinus function 
            (how long a period takes relatively to the path length)
        axis: int
            The axis on which the translation should be applied
        """

        for sample_index in range(number_of_samples):
            # Calculate the angle at index sample_index 
            angle = 1/frequency * (sample_index * 360.0) / number_of_samples
            # Yield a tuple of index and the sinus value of the angle, multiplied by the amplitude 
            yield sample_index, amplitude * math.sin(math.radians(angle))

    new_path = []
    new_translation = np.asarray([0.0, 0.0, 0.0])

    number_of_samples = len(cartesian_path)
    # For every pose, get the translation value of the sinus shaped movement
    for i, s in sin_generator(number_of_samples, amplitude, frequency):
        # Apply it only to one axis
        new_translation[axis] = s
        # Apply the translation to corresponding pose
        new_pose = cartesian_path.points[i].translate(new_translation)
        new_path.append(new_pose)
    return CartesianPath(new_path)

async def main():
    move_group = example_utils.get_move_group()
    end_effector = move_group.get_end_effector()

    pose = end_effector.get_current_pose()
    # Create a cartesian path of the same poses
    path = []
    for i in range(300):
        path.append(pose)
    cartesian_path = CartesianPath(path)

    # This applies a sinus trajectory an amplitude of 5 mm with 10 periods 
    # over the path on the y axis  
    target = add_sin_to_trajectory(cartesian_path=cartesian_path, 
                                amplitude=0.005, frequency=1/10, axis=1)
    # This applies a sinus trajectory an amplitude of 2 mm with 100 periods 
    # over the path on the z axis  
    target = add_sin_to_trajectory(target, amplitude=0.002, frequency=1/100, axis=2)
    # Move the end effector according to the now sinus shaped path
    await end_effector.move_poses_linear(target, max_deviation=0.05)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)
    try:
        loop.run_until_complete(main())
    finally:
        loop.close()
