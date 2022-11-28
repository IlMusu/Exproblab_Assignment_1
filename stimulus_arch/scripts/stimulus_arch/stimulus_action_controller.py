#!/usr/bin/env python

# Importing THREADING library to make system multithreaded
import threading

from stimulus_arch.stimulus_controller import StimulusController

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class StimulusActionController(StimulusController):
    '''
    This class extends the StimulusController to create a placeholder for a 
    controller that requires an ActionServer.
    '''
    
    def __init__(self):
        '''
        This is the constructor method for the StimulusActionController class.
        Initializes some variables for handling the ActionServer.
        '''
        StimulusController.__init__(self)
        # Some variables for handling the server
        self._goal = None
        self._result = None
        self._result_event = threading.Event()
        
        
    def _get_controller_method(self) :
        '''
        Returns:
            (function) : The method representing the execution mode.
        '''
        if self._execution_mode == 'MANUAL' :
            return self._manual_controller
        elif self._execution_mode == 'RANDOM' :
            return (lambda : None)
        return None
    
    
    def _start_action_server(self, server):
        '''
        Args:
            server (ActionServer) : The ActionServer related to this controller.
            
        This method starts the action server and stores it a in an internal
        variable so that it can be accessed by other methods.
        '''
        self._server = server
        self._server.start()
    
    
    def _action_callback(self, goal):
        '''
        Args:
            goal (Goal) : The requested goal.
        
        This is the callback template for the ActionServer:
        1. If the execution mode is MANUAL, it waits for user input to set
           the result of the requested goal. This is done via a Lock.
        2. If the execution mode is AUTOMATIC, the result is automatically
           computed by the controller.
        The it sets the computed result to the ActionServer.
        '''
        # Storing the goal to be used later by the controller method
        self._goal = goal
        # If the execution is MANUAL, the user has to insert the related command
        # to make the execution complete. Otherwise the action is handled automatically
        # by the random controller method.
        if self._execution_mode == 'MANUAL' :
            print(CGREEN+'A new action request is available.')
            print('The execution will not happen until you insert the related command.'+CRESET)
            # Waiting for the user to set the response
            self._result_event.clear()
            self._result_event.wait()
            self._result_event.clear()
        else :
            self._random_controller()
        # The result is now available
        self._server.set_succeeded(self._result)
        # Resetting the state
        self._goal = None
        self._result = None
        
 
    def _set_action_result(self, result):
        '''
        Args:
            result (ActionResult) : The result that is computed for a request.

        Releases the Lock for the result so that the action server callback can be
        resumed now that the result is fully computed.
        '''
        self._result = result
        self._result_event.set()


