#!/usr/bin/env python

# Importing AST library for safely evaluating strings
import ast
# The RANDOM library is necessary for debugging
import random

# Importing ROS library for python
import rospy
import actionlib
from robot_state_msgs.msg import ComputePathAction, ComputePathResult
from geometry_msgs.msg import Point

# Importing the helper for handling user commands
from stimulus_arch.command_helper import CommandHelper
from stimulus_arch.stimulus_action_controller import StimulusActionController

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class PlannerController(StimulusActionController):
    '''
    Creates an action server for:
        /follow_path (FollowPath)

    This class is a controller that handles the planning of a path.
    '''
    
    def __init__(self):
        '''
        This is the constructor method for the MotionController class.
        It initializes the /motion_controller node.
        '''
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
        This method creates and starts an ActionServer called /compute_path.
        And then starts the StimulusController.
        '''
        # Defining an action server to simulate computing the plan
        server = actionlib.SimpleActionServer('/compute_path', ComputePathAction, 
                            execute_cb=self._action_callback, auto_start=False)
        self._start_action_server(server)
        # Actually starting the controller
        StimulusActionController.start(self)
        
    
    def _manual_controller(self):
        '''
        This method is executed only if the controller is executed in MANUAL mode.
        It is used to register the commands that are necessary to for the communication 
        between the user and this controller.
        '''
        # Registering the command with the related callback
        self._commands = CommandHelper()
        self._commands.register_command(['plan', 'set'], self._plan_command)
        # Handling commands inserted by the user
        self._commands.listen_for_user_commands('exit')
    
    
    def _plan_command(self, args):
        '''
        Args:
            args (list) : The list of arguments for the command.

        This method is the callback for the "plan" command, it creates a path with
        the positions specified by the user. It puts the initial position as the
        first point of the path and the goal positions as the last point.
        '''
        # Obtaining the path inserted by the user
        try :
            argument = ast.literal_eval(''.join(args))
        except Exception :
            print(CGREEN+'The inserted path was not formatted correctly.'+CRESET)
            return
        # Computing path from array of coordinates
        path = [self._goal.start]
        for [x, y, z] in argument :
            path.append(Point(x, y, z))
        path.append(self._goal.goal)
        # Filling and setting the result
        self._set_result_path(path)
        
        
    def _random_controller(self):
        '''
        This method is executed only if the controller is execute in RANDOM mode.
        It fills the path with completely random points except the first and the last
        one which are the initial and goal positions.
        '''
        path = [self._goal.start]
        for i in range(5, random.randrange(5,10)):
            x = random.uniform(-10.0,10.0)
            y = random.uniform(-10.0,10.0)
            z = random.uniform(-10.0,10.0)
            path.append(Point(x, y, z))
        path.append(self._goal.goal)
        # Filling and setting the result
        self._set_result_path(path)
    
    
    def _set_result_path(self, path):
        '''
        Args:
            path (list) : A list of Points which compose the path.
            
        Sets the path as result of the ActionServer and logs some info.
        '''
        result = ComputePathResult()
        result.path = path
        self._set_action_result(result)
        rospy.loginfo(CYELLOW+'The computed path has been published.'+CRESET)
        


if __name__ == "__main__" :
    planner_controller = PlannerController()
    planner_controller.start()
    rospy.spin()

