""" 
This module contain some utility functions to get and format jogging feedback
"""

from example_07.jogging_client import JoggingClient, JoggingClientFeedbackState, JoggingErrorCode

def callback_function(state: JoggingClientFeedbackState):
    """
    This is a simple callback function to demonstrate the use of callback functions.

    In the context of this example, we just print the error code.
    """

    # only print the error code when it is not OK
    if not state.error_code ==  JoggingErrorCode.OK:
        print("Joggin Error occurred: {}".format(state.error_code))


