"""
This module uses pynput to give the user some control over the jogging


"""

from threading import Timer, Event
from threading import Thread, Lock
import time

import numpy as np
from pynput import keyboard

from example_07.jogging_client import JoggingClient
from example_07.twist import Twist

# Simple decorator 
def synchronized(function):
    def wrapper(self, *args, **kwargs):
        with self._mutex:
            return function(self, *args, **kwargs)
    return wrapper

class Ticker(Thread):
    """ Repeats a callback when active"""

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

    def __init__(self, jogging_client: JoggingClient, frequency: float=50):
        self._jogging_client = jogging_client
        self._keyboard_listener = KeyboardListener(self)
        self._frequency = frequency
        self._stop_event = Event()
        self._ticker = Ticker(self._frequency, self._on_tick, self._stop_event)
        self._mutex = Lock()
        self._forward = False 
        self._twist = Twist()
        self._zero_twist = Twist()
        self._sending = False

    def thread_start(self):
        print("Start thread")
        self._ticker.start()
        self._keyboard_listener.start()
        self._stop_event.set()
        self._ticker.join()

    # To avoid simultaneous chaning of the twist, lock this function
    @synchronized
    def _update_twist(self, linear: np.array):
        self._twist = Twist(linear=linear)


    def forward_begin(self):
        if not self._forward:
            print("ONWARDS!")
            """Begin going forward"""
            self._forward = True
            linear = self._twist.linear
            linear[2] = linear[2] + 1 
            self._update_twist(linear=linear)

    def forward_end(self):
        """Begin going forward"""
        self._forward = False
        linear = self._twist.linear
        # update z axis
        linear[2] = linear[2] - 1 
        self._update_twist(linear=linear)


    def backwards_begin(self):
        if not self._forward:
            print("BACKWARDS!")
            """Begin going forward"""
            self._forward = True
            linear = self._twist.linear
            linear[2] = linear[2] - 1 
            self._update_twist(linear=linear)

    def backwards_end(self):
        """Begin going forward"""
        self._forward = False
        linear = self._twist.linear
        # update z axis
        linear[2] = linear[2] + 1 
        self._update_twist(linear=linear)


    def _on_tick(self):
#        print(self._twist)
        if not self._twist == self._zero_twist:
            self._sending = True
            self._jogging_client.send_twist(self._twist)
            print("lets twist again")
        else:
            if self._sending:
                print("one last time") 
                # Send one last time to stop operation
                self._sending = False
                self._jogging_client.send_twist(self._twist)

class KeyboardListener(object):
    """ 
    A class that listens to some keys an do calls to an interface when they are pressed.
    
    Keeps track of keys being pressed and released, so that keeping pressed a key 
    does only call the appropriate function once, as do releasing it.
    """

    # Dependency injection 
    def __init__(self, jogging_interface):
        self._jogging_interface = jogging_interface

    def on_press(self, key):
        try:        
            key_val = key.char
            if key_val == "w":
                self._jogging_interface.forward_begin()
            if key_val == "s":
                self._jogging_interface.backwards_begin()
        except AttributeError:
            None

    def on_release(self,  key):
        try:        
            key_val = key.char
            if key_val == "w":
                self._jogging_interface.forward_end()
            if key_val == "s":
                self._jogging_interface.backwards_end()
        except AttributeError:
            None
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def start(self):
        # Collect events until released
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
                listener.join()


