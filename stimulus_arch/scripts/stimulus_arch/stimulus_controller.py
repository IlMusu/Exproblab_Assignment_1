#!/usr/bin/env python

# Importing ROS library for python
import rospy

class StimulusController(object):
    '''
    ROS Parameters:
        - /execution_mode (string): The execution mode of this controller.

    |  This is an abstract to create a placeholder controller for developing and
       testing a ROS architecture. It provides two execution modes:
    |  - MANUAL: The user controls the controller via the terminal.
    |  - RANDOM: The controller generates random data.
    '''
    
    def __init__(self):
        '''
        This is the constructor method for the StimulusController class.
        Gets the current execution mode from the ROS Parameter Server.
        '''
        self._execution_mode = rospy.get_param('/execution_mode', 'MANUAL')
        
    
    def start(self):
        '''
        This method starts the StimulusController in the defined execution mode.
        Before starting, writes in console all the commands that the user can
        utilize for interacting with this controller.
        '''
        # Starting the controller in the specified mode
        target_controller=self._get_controller_method()
        if target_controller == None :
            rospy.logerr('Invalid execution mode ('+self._execution_mode+').')
            return
        # Actually starting the controller in the execution mode
        self._explain_commands()
        target_controller()
    

    def _get_controller_method(self) :
        '''
        Returns:
            (function) : The method representing the execution mode.
        '''
        if self._execution_mode == 'MANUAL' :
            return self._manual_controller
        elif self._execution_mode == 'RANDOM' :
            return self._random_controller
        return None
    
    
    def _explain_commands(self):
        '''
        This methods writes all the commands that the user can utilize for
        interacting with this controller.
        '''
        pass


    def _manual_controller(self):
        '''
        This method executes the MANUAL execution mode.
        '''
        pass
    
    
    def _random_controller(self):
        '''
        This method executes the RANDOM execution mode.
        '''
        pass


