"""
This example, we combine the keyboard listener of pynput with some basic jogging.

To use this example, you have to install pynput and python3-tk inside the docker container.

After starting rosvita, enter 
    $sudo pip install pynput
    $sudo apt install python3-tk
"""

import numpy as np
import math

from threading import Timer, Event, Thread, Lock
import time 

from pynput import keyboard

from xamla_motion.data_types import Pose
from xamla_motion.motion_client import MoveGroup
from xamla_motion.world_view_client import WorldViewClient

import example_utils
from example_07.example_07_jogging_feedback import callback_function as feedback_function

from example_07.jogging_client import JoggingClient
from example_07.twist import Twist

from xamla_motion.xamla_motion_exceptions.exceptions import ServiceException

def add_generated_folder(world_view_client: WorldViewClient, world_view_folder: str) -> None:
    """ Adds a folder to world view, deletes content if existent"""

    try:
        # delete folder if it already exists
        world_view_client.remove_element("generated", world_view_folder)
    except Exception as e:
        None
    world_view_client.add_folder("generated", world_view_folder)

def synchronized(function):
    """Simple decorator to add atomic access to function """
    def wrapper(self, *args, **kwargs):
        with self._mutex:
            return function(self, *args, **kwargs)
    return wrapper


class Ticker(Thread):
    """Repeats a callback in a given frequency when active"""

    def __init__(self, frequency: float, callback, stop_event):
        Thread.__init__(self) 
        self._stop_event = stop_event
        self._running = True
        self._frequency = frequency
        self._callback = callback
        
    def run(self):
        # wait for a given time, then call the callback function. Then wait again

        while not self._stop_event.wait(1/self._frequency):
            self._callback()


class JoggingKeyboardInterface(object):
    """     
    This class manages the input of a KeyListener instance and the calls to the 
    jogging client .

    For that, two threads are started. 
    One is the key listener of pynput, which keeps track of the key pressed and released. 
    The other is a ticker thread, which manages the calls to the jogging client by calling 
    the _on_tick function in a certain frequency, which calls the functions of the jogging 
    client.
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
        self._stop_event = Event()
        self._ticker = Ticker(frequency, self._on_tick, self._stop_event)
        # A lock object, which is used by the synchronized decorator 
        self._mutex = Lock()

        self._current_frame = "_gripper_right_tool0"

        self._linear = np.array([0.0,0.0,0.0])
        self._angular = np.array([0.0,0.0,0.0])
        self._sending = False
        self._pose_counter = 1

    def thread_start(self) -> None:
        """
        This function starts the ticker thread and the keyboard listener thread.
        The start function of the KeyboardListener instance returns when it is stopped, 
        hence when pressing the "escape" key.  
        """

        print("Start thread")
        self.show_help()
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
    def update_linear(self, index: int , value: float) -> None:
        """
        Updates the linear component of the twist, one entry at a time 
        
        Parameters
        ----------
        index : int
            The index of the axis, where x=0, y=1, z=2
        value : float
            The value to be set.
        """

        self._linear[index] = value

    @synchronized
    def update_angular(self, index: int, value: float) -> None:
        """
        Updates the angular component of the twist, one entry at a time 
        
        Parameters
        ----------
        index : int
            The index of the axis, where x=0, y=1, z=2
        value : float
            The value to be set.
        """

        self._angular[index] = value

    def change_frame(self) -> None:
        """ 
        Changes the current frame to the next one
        """

        self._current_frame = self._get_next_frame()
        print("Changed current frame to {}".format(self._current_frame))

    def save_pose(self) -> None:
        """
        Saves the current pose of the end effector to world view in a folder generated folder.
        """
        
        pose_id = "pose_{}".format(self._pose_counter) 
        print("Save current Pose {} to world view client".format(pose_id))
        pose = self._current_pose()
        self._world_view_client.add_pose(pose_id, 
            "{}/generated/".format(self._world_view_folder), 
            pose)
        self._pose_counter += 1

    def update_velocity_scaling(self, delta: float) -> None:
        """
        Updates the velocity scaling
        Parameters
        ----------
        delta : float
            The change applied to the velocity scaling 
        """

        current_velocity =  self._jogging_client.get_velocity_scaling()
        current_velocity += delta 
        current_velocity = max(0.0, min(1.0, current_velocity))
        try:
            self._jogging_client.set_velocity_scaling(current_velocity)
            print("Velocity scaling: {}".format(self._jogging_client.get_velocity_scaling()))
        except ServiceException as e:
            print("Could not adjust velocity scaling, since service unreachable.")

    def show_help(self) -> None:
        """
        Prints help
        """

        print("----------------------------------------------")
        print("Move in x direction     :  a,d ")
        print("Move in y direction     :  page_up,page_down ")
        print("Move in z direction     :  x,y")
        print("Roll around x-axis      :  up,down")
        print("Roll around y-axis      :  q,e ")
        print("Roll around z-axis      :  right,left ")
        print("Change velocity scaling :  +,- ")
        print("Toggle world frame      :  m ")
        print("Show this help          :  h ")
        print("Store current pose      :  enter")
        print("Stop keyboard listener  :  escape ")
        print("----------------------------------------------")

    def _get_next_frame(self) -> str:
        """Just toggle between world and gripper frame""" 

        if self._current_frame == "world":
            return "_gripper_right_tool0"
        else:
            return "world" 

    def _current_pose(self) -> Pose:
        """
        Returns the current pose of the end effector
        """

        joint_values = self._move_group.get_current_joint_positions()
        return self._move_group.get_end_effector().compute_pose(joint_values)

    def _current_twist(self) -> Twist:
        """
        Based on current state, create a Twist instance
        """

        return Twist(linear = self._linear,
            angular = self._angular, 
            frame_id = self._current_frame)

    def _on_tick(self) -> None:
        """
        Called by the Ticker instance in a certain frequency, to see if calls to 
        the jogging client have to be made.
        """

        current_twist = self._current_twist()
        if not current_twist == Twist():
            self._sending = True
            self._jogging_client.send_twist(current_twist)
        elif self._sending: 
            # Send one last time to stop operation
            self._sending = False
            self._jogging_client.send_twist(current_twist)


class KeyboardListener(object):
    """ 
    A class that listens to some keys an calls functions of a JoggingKeyboardInterface
    instance when they are pressed.

    This dictionary self._key_bindings binds the pushing and releasing of a key 
    to a function.
    These functions are all called with a parameter "pressed", which indicate
    the current state of the key(True: pressed, False: released).
    Depending on the "pressed" state, the function binded to the key is called 
    either with different parameters (wrapped by self._call_always), only when 
    pressed (wrapped by self._call_when_pressed) or when released (wrapped by 
    self._call_when_released).
    """

    def __init__(self, jogging_interface):
        self._jogging_interface = jogging_interface

        self._key_bindings = {
            "w" : self._call_always(    # move in z direction
                func=self._jogging_interface.update_linear, 
                args_pressed=(2, 1),    # "func" is call with this when button is pressed (repeatedly)
                args_released=(2, 0)),  # "func" is call with this when button is released (once)
            "s" : self._call_always(    # move in -z direction
                func=self._jogging_interface.update_linear, 
                args_pressed=(2, -1), 
                args_released=(2, 0)),
            "e" : self._call_always(    # roll around y axis
                func=self._jogging_interface.update_angular, 
                args_pressed=(1, 32), 
                args_released=(1, 0)),
            "q" : self._call_always(    # roll around -y axis
                func=self._jogging_interface.update_angular,
                args_pressed=(1, -32), 
                args_released=(1, 0)),
            "d" : self._call_always(    # move in x direction
                func=self._jogging_interface.update_linear,
                 args_pressed=(0, 1), 
                 args_released=(0, 0)),
            "a" : self._call_always(    # move in -x direction
                func=self._jogging_interface.update_linear, 
                args_pressed=(0, -1), 
                args_released=(0, 0)),
            "page_down" : self._call_always(    # move in y direction
                func=self._jogging_interface.update_linear,
                 args_pressed=(1, 1), 
                 args_released=(1, 0)),
            "page_up" : self._call_always(    # move in -y direction
                func=self._jogging_interface.update_linear, 
                args_pressed=(1, -1), 
                args_released=(1, 0)),
            "down" : self._call_always(    # roll around x axis
                func=self._jogging_interface.update_angular,
                 args_pressed=(0, 32), 
                 args_released=(0, 0)),
            "up" : self._call_always(    # roll around x axis
                func=self._jogging_interface.update_angular, 
                args_pressed=(0, -32), 
                args_released=(0, 0)),
            "right" : self._call_always(    # roll around -z axis
                func=self._jogging_interface.update_angular,
                 args_pressed=(2, 32), 
                 args_released=(2, 0)),
            "left" : self._call_always(    # roll around -z axis
                func=self._jogging_interface.update_angular, 
                args_pressed=(2, -32), 
                args_released=(2, 0)),
            "m" : self._call_when_released( # change frame
                func=self._jogging_interface.change_frame),
            "+" : self._call_when_pressed(  # Increase velocity
                func=self._jogging_interface.update_velocity_scaling, 
                args_pressed=(1/32,)),
            "-" : self._call_when_pressed(  # Decrease velocity
                func=self._jogging_interface.update_velocity_scaling, 
                args_pressed=(-1/32,)),
            "enter" : self._call_when_released(  # Save pose to world view
                func=self._jogging_interface.save_pose),
            "h" : self._call_when_released(  # Show help
                func=self._jogging_interface.show_help)
        }

    def _call_when_pressed(self, func, args_pressed=()):
        """ 
        wraps a given function by calling it with the arguments when key has been pressed
        """
        return lambda pressed : func(*args_pressed) if pressed else None
    
    def _call_when_released(self, func, args_released=()):
        """ 
        wraps a given function by calling it with the arguments when key has been pressed
        """
        return lambda pressed : func(*args_released)  if not pressed else None

    def _call_always(self, func, args_pressed, args_released=()):
        """ 
        wraps a given function by calling it with the args_pressed when pressed is true, 
        and with args_released when pressed is false. 
        """
        return lambda pressed : func(*args_pressed) if pressed else func(*args_released)

    def on_press(self, key):
        """
        This function is called when a button is pressed
        """
        return self._handle_key(key, True) 

    def on_release(self,  key):
        """
        This function is called when a button is released
        """
        return self._handle_key(key, False)

    def _handle_key(self, key, pressed: bool):
        """ 
        This function handles the pressing or releasing of a key

        For the key, the appropriate function mapped by self._key_bindings is called
        """

        key_val = None
        try:      
            key_val = key.char
        except AttributeError:
            None
        # Map special keys to strings as id
        if key == keyboard.Key.up:
            key_val = "up"
        elif key == keyboard.Key.down:
            key_val = "down"
        if key == keyboard.Key.right:
            key_val = "right"
        elif key == keyboard.Key.left:
            key_val = "left"
        elif key == keyboard.Key.enter:
            key_val = "enter"
        elif key == keyboard.Key.page_up:
            key_val = "page_up"
        elif key == keyboard.Key.page_down:
            key_val = "page_down"
        elif key == keyboard.Key.esc:
            print("Stop keyboard control")
            # Returning False to the keyboard.Listener instance stops the thread
            return False
        if key_val and key_val in self._key_bindings.keys():
            self._key_bindings[key_val](pressed)

    def start(self):
        """
        Starts the keyboard listener.
        This function returns when the listener thread finishes.
        """

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

    interface = JoggingKeyboardInterface(jogging_client, move_group, world_view_client)
    interface.thread_start()

    # Stop tracking
    jogging_client.stop()
    # Unregister the feedback function
    jogging_client.unregister(feedback_function)

if __name__ == '__main__':
    main()

