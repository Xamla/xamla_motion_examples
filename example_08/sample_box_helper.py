"""
Helper functions to create a SampleBox instance

"""

from pyquaternion import Quaternion
import numpy as np

from  xamla_motion.trajectory_caching import SampleBox
from xamla_motion.data_types import Pose

def get_sample_box(mid_point: Pose) -> SampleBox:
    """
    This function creates a sample box.
    A sample box contains a set of translation vectors and
    a set of quaternions.
    create_trajectory_cache, when called with the box either as target (one to many)
    or as source (many to on), tries to get trajectories to/from every pose which 
    can be created out of all possible combination of rotation and translation
    defined by the sample box.
    """
    
    quaternions = []
    for i in range(7):
        a = (np.pi/4)*i
        q = Quaternion(matrix=np.array([[np.cos(a), -np.sin(a), 0],
                                        [np.sin(a), np.cos(a), 0],
                                        [0, 0, 1]], dtype=float))

        r = mid_point.quaternion * q
        quaternions.append(r)

    sample_rect = SampleBox(origin=mid_point,
                            size=[0.2, 0.2, 0],
                            resolution=[0.05, 0.05, 0.05],
                            quaternions=quaternions)
    return sample_rect

if __name__ == '__main__':
    pose = Pose.from_transformation_matrix(np.eye(4))
    print(get_sample_box(pose).sample_positions)