# twist.py
#
# Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#!/usr/bin/env python3

import numpy as np

class Twist(object):
    """ 
    Twist contains linear velocity in m/s and Angular velocity in rad/s decribed 
    in a specific ROS TF frame

    """

    def __init__(self, linear=None, angular=None, frame_id=""):
        """
        Initialization of the twist class

        Parameters
        ----------
        linear : convertable to numpy array of shape (3,) or None
            The linear velocity in m/s
        angular: convertable to numpy array of shape (3,) or None
            The angular velocity in rad/s
        frame_id : str (optinal default = '')
            Name of the coordinate system the pose is defined

        Returns
        ------
        Twist
            An instance of class Twist
        """
        self.__linear = linear
        if not angular:
            self._linear = np.array([0.0,0.0,0.0])
        self.__angular = angular
        if not angular:
            self._angular = np.array([0.0,0.0,0.0])
        self.__frame_id = frame_id

    @property
    def frame_id(self):
        """
        frame_id : str (readonly)
            Id of the coordinate system / frame
        """
        return self.__frame_id

    @property
    def linear(self):
        """
        translation : numpy.array((3,) dtype=floating) (readonly)
            numpy row array of size 3 which describes the linear velocity
        """
        return self.__linear

    @property
    def angular(self):
        """
        translation : numpy.array((3,) dtype=floating) (readonly)
            numpy row array of size 3 which describes the angular velocity
        """
        return self.__angular