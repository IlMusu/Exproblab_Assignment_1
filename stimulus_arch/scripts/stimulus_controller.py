#!/usr/bin/env python

# Imporing ROS library for python
import rospy

class StimulusController(object):
    
    def __init__(self):
        '''
        This is the constructor method for the StimulusController class.
        Gets the current execution mode from the Parameter Server.
        '''
        self._execution_mode = rospy.get_param('/execution_mode', 'MANUAL')
        
    
    def start(self):
        '''
        This method starts the StimulusController is the execution mode.
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
        if self._execution_mode == 'MANUAL' :
            return self._manual_controller
        elif self._execution_mode == 'RANDOM' :
            return self._random_controller
        return None
    
    
    def _explain_commands(self):
        pass


    def _manual_controller(self):
        pass
    
    
    def _random_controller(self):
        pass


