""" 
This module contain some utility functions to get and format jogging feedback
"""

import threading


from xamla_motion.jogging_client import JoggingClient, JoggingClientFeedbackState, JoggingErrorCode

msg = ""
timer = threading.Timer(1.0, lambda x : None)
blind = False

def callback_function(state: JoggingClientFeedbackState):
    """
    This is a simple callback function to demonstrate the use of callback functions.

    In the context of this example, we just print the error code.
    """
    def callback():
        """ resets blind to false"""
        global blind
        blind = False

    global msg
    global timer
    global blind
    # only print the error code when it is not OK
    if not state.error_code ==  JoggingErrorCode.OK:
        new_msg = "Jogging Error occurred: {}".format(state.error_code)
        if not new_msg == msg or not blind:
            msg = new_msg
            print(msg)
            blind  = True
            timer = threading.Timer(1.0, callback)
            timer.start()

            
                

