#!/usr/bin/env python

# Importing AST library for safely evaluating strings
import ast
# The RANDOM library is necessary for debugging
import random

# Imporing ROS library for python
import rospy
import actionlib
from robot_state_msgs.msg import ComputePathAction, ComputePathResult
from geometry_msgs.msg import Point

# Importing the helper for handling user commands
from command_helper import CommandHelper
from stimulus_action_controller import StimulusActionController

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class PlannerController(StimulusActionController):
    
    def __init__(self):
        StimulusActionController.__init__(self)
        # Initializing the ROS node
        rospy.init_node('planner_controller', log_level=rospy.INFO)


    def _explain_commands(self):
        '''
        This method is used to print in console the available commands in console.
        Actually, the commands are shown to the user only if the controller is set
        in MANUAL execution mode, because in RANDOM mode the commands are not used.
        '''
        rospy.loginfo(CGREEN+'Planner controller is started in '+self._execution_mode+' mode.'+CRESET)
        # Informing user how to control the battery
        if self._execution_mode == 'MANUAL' :
            print('You can interact with the planner controller with the commands:')
            print('- '+CGREEN+'plan set <plan>'+CRESET+' :')
            print('\t 1. <plan> (list) : The list of points that compose the plan.')
            print('                      Ex: [[0, 0, 0], [0, 0.1, 0], [0, 0.2, 0]] .')


    def start(self):
        '''
        This method start the controller in the specified execution mode.
        '''
        # Defining an action server to simulate computing the plan
        self._move_srv = actionlib.SimpleActionServer('/compute_plan', ComputePathAction, 
                            execute_cb=self._action_callback, auto_start=False)
        self._move_srv.start()
        # Actually starting the controller
        StimulusActionController.start(self)
        
    
    def _manual_controller(self):
        '''
        |  This method is executed only if the controller is execute in MANUAL mode.
        |  It is used to register the commands that are necessary to for the 
        |  communication between the user and this controller.
        '''
        # Registering the command with the related callback
        self._commands = CommandHelper()
        self._commands.register_command(['plan', 'set'], self._plan_command)
        # Handling commands inserted by the user
        self._commands.listen_for_user_commands('exit')
    
    
    def _plan_command(self, args):
        # Obtaining the path inserted by the user
        argument = ast.literal_eval(''.join(args))
        # Computing path from array of coordinates
        path = [self._goal.start]
        for [x, y, z] in argument :
            path.append(Point(x, y, z))
        path.append(self._goal.goal)
        # Filling and setting the result
        result = ComputePathResult()
        result.path = path
        self._set_action_result(result)
        
        
    def _random_controller(self):
        '''
        This method is executed only if the controller is execute in RANDOM mode.
        It fills the path with complitely random points except the first and the last
        one which are the initial and goal points.
        '''
        path = [self._goal.start]
        for i in range(5, random.randrange(5,10)):
            path.append(Point(random.uniform(-10.0,10.0)))
        path.append(self._goal.goal)
        # Filling and setting the result
        result = ComputePathResult()
        result.path = path
        self._set_action_result(result)
    


if __name__ == "__main__" :
    planner_controller = PlannerController()
    planner_controller.start()
    rospy.spin()

