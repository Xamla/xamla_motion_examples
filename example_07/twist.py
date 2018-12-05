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
import geometry_msgs.msg as geometry_msgs

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
         Raises
        ------
        TypeError : type mismatch
            If linear or angular vector could not be converted to a numpy 
            array of shape (3,) with d type floating  
            If frame_id is not of type str
        
        """
        # Check for None
        if not linear:
            self._linear = np.array([0.0,0.0,0.0])
        if not angular:
            self._angular = np.array([0.0,0.0,0.0])

        try:
            self.__linear = np.fromiter(linear, float)
            if self.__linear.shape[0] != 3:
                raise TypeError('provided linear vector is not'
                                 ' convertable to a numpy vector of size (3,)')
        except TypeError as exc:
            raise exc

        try:
            self.__angular = np.fromiter(angular, float)
            if self.__angular.shape[0] != 3:
                raise TypeError('provided angular vector is not'
                                 ' convertable to a numpy vector of size (3,)')
        except TypeError as exc:
            raise exc
 
        if not isinstance(frame_id, str):
            raise TypeError('frame_id is not of expected type str')
        self.__frame_id = frame_id


    @classmethod
    def from_twiststamped_msg(cls, msg):
        """
        Initialize Pose from ROS twiststamped message

        Parameters
        ----------
        msg : TwistStamped from ROS geometry_msgs
            TwistStamped message 

        Returns
        -------
        Twist
            Instance of Twist generated from TwistStamped message

        Raises
        ------
        TypeError
            If msg is not of type TwistStamped
        """
        if not isinstance(msg, geometry_msgs.TwistStamped):
            raise TypeError('msg is not of expected type TwistStamped')

        frame_id = msg.header.frame_id

        if not frame_id:
            frame_id = ""

        return cls.from_twist_msg(msg.twist, frame_id)

    @classmethod
    def from_twist_msg(cls, msg, frame_id = ""):
        """
        Initialize Pose from ROS twist message

        Parameters
        ----------
        msg : Twist from ROS geometry_msgs
            Twist message 

        Returns
        -------
        Twist
            Instance of Twist generated from Twist message

        Raises
        ------
        TypeError
            If msg is not of type Twist
        """
        if not isinstance(msg, geometry_msgs.Twist):
            raise TypeError('msg is not of expected type Twist')

        twist_msg = msg.twist
        
        twist = cls.from_twist_msg

        linear = np.fromiter([msg.linear.x,
                                msg.linear.y,
                                msg.linear.z],
                                float)

        angular =  np.fromiter([msg.angular.w,
                                msg.angular.x,
                                msg.angular.y,
                                msg.angular.z])

        frame_id = msg.header.frame_id

        if not isinstance(frame_id, str):
            raise TypeError('frame_id is not of expected type str')
        return cls(linear, angular, frame_id)



    def to_twiststamped_msg(self):
        """
        Creates an instance of the ROS message TwistStamped

        Returns
        ------
            Instance of ROS message TwistStamped (seq and time are not set)
            docs.ros.org/kinetic/api/geometry_msgs/html/msg/TwistStamped.html
        """
        twist_stamped = geometry_msgs.TwistStamped()
        twist_stamped.header.frame_id = self.__frame_id
        twist_stamped.twist = self.to_twist_msg()

        return twist_stamped

    def to_twist_msg(self):
        """
        Creates an instance of the ROS message Twist

        Returns
        ------
            Instance of ROS message Twist
            docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html
        """

        twist = geometry_msgs.Twist()

        twist.linear.x = self.__linear[0]
        twist.linear.y = self.__linear[1]
        twist.linear.z = self.__linear[2]

        twist.angular.w = self.__angular[0]
        twist.angular.x = self.__angular[1]
        twist.angular.y = self.__angular[2]
        return twist

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

