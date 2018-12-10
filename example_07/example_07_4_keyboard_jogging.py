"""
This example shows how to rotate the torso joint while keeping the end effectors
at certain poses 
"""

import numpy as np
import math

import time 

import copy

from xamla_motion.world_view_client import WorldViewClient

from xamla_motion.data_types import Pose, JointSet, JointValues, JointPath
from xamla_motion.motion_client import MoveGroup

import example_utils

from example_07.jogging_client import JoggingClient, Twist

#from example_07.example_07_keyboard_control import JoggingInterface
from example_07.example_07_jogging_feedback import callback_function as feedback_function



from threading import Timer, Event
from threading import Thread, Lock
import time

import numpy as np
from pynput import keyboard

from xamla_motion.data_types import Pose
from xamla_motion.motion_client import MoveGroup
from xamla_motion.world_view_client import WorldViewClient

from example_07.jogging_client import JoggingClient
from example_07.twist import Twist

def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a folder to world view, deletes content if existand"""

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)

# Simple decorator to apply atomic access to function 
def synchronized(function):
    def wrapper(self, *args, **kwargs):
        with self._mutex:
            return function(self, *args, **kwargs)
    return wrapper

class Ticker(Thread):
    """ Repeats a callback in a given frequency when active"""

    def __init__(self, frequency: float, callback, stop_event):
        Thread.__init__(self) 
        self._stop_event = stop_event
        self._running = True
        self._frequency = frequency
        self._callback = callback
        
    def run(self):
        while not self._stop_event.wait(1/self._frequency):
            self._tick()

    def _tick(self):
        self._callback()

class JoggingInterface(object):
    """ 
    Has a thread which calls every 1/self._frequency appropriate functions of JoggingClient

    """

    def __init__(self, jogging_client: JoggingClient, 
            move_group: MoveGroup,
            world_view_client: WorldViewClient, 
            frequency: float=50):
        self._jogging_client = jogging_client
        self._move_group = move_group
        self._world_view_folder = "example_07_jogging"
        self._world_view_client = world_view_client

        self._keyboard_listener = KeyboardListener(self)
        self._frequency = frequency
        self._stop_event = Event()
        self._ticker = Ticker(self._frequency, self._on_tick, self._stop_event)
        self._mutex = Lock()

        self._current_frame = "_gripper_right_tool0"

        self._linear = np.array([0.0,0.0,0.0])
        self._angular = np.array([0.0,0.0,0.0])
        self._sending = False
        self._pose_counter = 1

    def thread_start(self):
        print("Start thread")
        add_generated_folder(self._world_view_client, self._world_view_folder)
        self.save_pose()
        self._ticker.start()
        # This keeps running until listener is finished 
        self._keyboard_listener.start()
        # clean up

        self._stop_event.set()
        self._ticker.join()
        self._world_view_client.remove_element("generated", self._world_view_folder)

    @synchronized
    def update_linear(self, index, value):
        """Update the linear component of the twist, one entry at a time 
        
        
        
        """
        self._linear[index] = value

    @synchronized
    def update_angular(self, index, value):
        self._angular[index] = value

    def change_world_frame(self):
        self._current_frame = self._get_next_frame()
        print("Changed current frame to {}".format(self._current_frame))

    def save_pose(self):
        print("Save current pose to world view client")
        pose_id = "pose_{}".format(self._pose_counter) 
        pose = self._current_pose()
        self._world_view_client.add_pose(pose_id, 
            "{}/generated/".format(self._world_view_folder), 
            pose)
        self._pose_counter += self._pose_counter



    def increase_velocity(self):
        None

    def decrease_velocity(self):
        None

    def _get_next_frame(self):
        """Just toggle between world and gripper frame""" 
        if self._current_frame == "world":
            return "_gripper_right_tool0"
        else:
            return "world" 

    def _current_pose(self) -> Pose:
        joint_values = self._move_group.get_current_joint_positions()
        return self._move_group.get_end_effector().compute_pose(joint_values)

    def _current_twist(self) -> Twist:
        return Twist(linear = self._linear,
            angular = self._angular, 
            frame_id = self._current_frame)

    def _on_tick(self):
        current_twist = self._current_twist()
        if not current_twist == Twist():
            self._sending = True
            self._jogging_client.send_twist(current_twist)
        else:
            if self._sending: 
                # Send one last time to stop operation
                self._sending = False
                self._jogging_client.send_twist(current_twist)

class KeyboardListener(object):
    """ 
    A class that listens to some keys an do calls to an interface when they are pressed.
    
    Keeps track of keys being pressed and released, so that keeping pressed a key 
    does only call the appropriate function once, as do releasing it.
    """

    # Dependency injection 
    def __init__(self, jogging_interface):
        self._jogging_interface = jogging_interface
        # This dictionary binds the pushing and releasing of a key to a function 
        # # of the jogging client, defining for example which axis should be 
        # updated when pressed (value == 1) or released (value == 0)
        self._key_bindings = {
            "w" : lambda value : self._jogging_interface.update_linear(2, value),
            "s" : lambda value : self._jogging_interface.update_linear(2, -value),
            "e" : lambda value : self._jogging_interface.update_angular(1, value*10),
            "q" : lambda value : self._jogging_interface.update_angular(1, -value*10),
            "d" : lambda value : self._jogging_interface.update_linear(0, value),
            "a" : lambda value : self._jogging_interface.update_linear(0, -value),
            "m" : lambda value : self._jogging_interface.change_world_frame() if value == 0 else None,
            "+" : lambda value : self._jogging_interface.increase_velocity() if value == 1 else None,
            "-" : lambda value : self._jogging_interface.decrease_velocity() if value == 1 else None,
            "enter" : lambda value : self._jogging_interface.save_pose() if value == 0 else None
        }

    def on_press(self, key):
        try:      
            key_val = key.char
            if key_val in self._key_bindings.keys():
                # Call the function
                self._key_bindings[key_val](1)

        except AttributeError:
            None

    def on_release(self,  key):
        try:        
            key_val = key.char
            if key_val in self._key_bindings.keys():
                # Call the function
                self._key_bindings[key_val](0)
        except AttributeError:
            None
        if key == keyboard.Key.esc:
            # Stop listener
            print("Stop keyboard control")
            return False
        if key == keyboard.Key.enter:
            self._key_bindings["enter"](0)

    def start(self):
        # Collect events until released
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
                listener.join()


def main():
    world_view_folder = "example_07_jogging"
    jogging_client = JoggingClient()

    world_view_client = WorldViewClient()
    move_group = example_utils.get_move_group()
    current_pose = move_group.get_end_effector().compute_pose(move_group.get_current_joint_positions())

    move_group_name = "/sda10f/right_arm_torso"
    jogging_client.set_move_group_name(move_group_name)
    

    # register feedback function, to get alerted when an error occurs
    jogging_client.register(feedback_function)
    #Begin tracking
    jogging_client.start()

    interface = JoggingInterface(jogging_client, move_group, world_view_client)

    interface.thread_start()

    # Stop tracking
    jogging_client.stop()
    # Unregister the feedback function
    jogging_client.unregister(feedback_function)


if __name__ == '__main__':
    main()

