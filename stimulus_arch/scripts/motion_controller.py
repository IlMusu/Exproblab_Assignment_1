#!/usr/bin/env python

# Importing AST library for safely evaluating strings
import ast
# The RANDOM library is necessary for debugging
import random

# Imporing ROS library for python
import rospy
import actionlib
from robot_state_msgs.msg import FollowPathAction, FollowPathResult
from geometry_msgs.msg import Point

# Importing the helper for handling user commands
from command_helper import CommandHelper
from stimulus_action_controller import StimulusActionController

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class MotionController(StimulusActionController):
    '''
    Creates an action server for:
        /follow_path (FollowPath)

    This class is a controller that handles the movement of the robot.
    In particular, it makes the robot follow a predefined path.
    '''
    
    def __init__(self):
        '''
        This is the constructor method for the MotionController class.
        It initializes the /motion_controller node.
        '''
        StimulusActionController.__init__(self)
        # Initializing the ROS node
        rospy.init_node('motion_controller', log_level=rospy.INFO)


    def _explain_commands(self):
        '''
        This method is used to print in console the available commands in console.
        Actually, the commands are shown to the user only if the controller is set
        in MANUAL execution mode, because in RANDOM mode the commands are not used.
        '''
        rospy.loginfo(CGREEN+'Motion controller is started in '+self._execution_mode+' mode.'+CRESET)
        # Informing user how to control the battery
        if self._execution_mode == 'MANUAL' :
            print('You can interact with the motion controller with the commands:')
            print('- '+CGREEN+'move <time> <delta>'+CRESET+' :')
            print('\t 1. <time> (int) : The time in seconds it takes for the robot to move.')
            print('\t 2. <delta> (float) : The error with respect to the requested goal.')


    def start(self):
        '''
        This method creates and starts an ActionServer called /follow_path.
        And then starts the StimulusController.
        '''
        # Defining an action server to simulate computing the plan
        server = actionlib.SimpleActionServer('/follow_path', FollowPathAction, 
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
        self._commands.register_command(['move'], self._move_command)
        # Handling commands inserted by the user
        self._commands.listen_for_user_commands('exit')
    
    
    def _move_command(self, args):
        '''
        Args:
            args (list) : The list of arguments for the command.

        This method is the callback for the "move" command, it makes the robot move
        with the related method.
        '''
        # Parsing the arguments
        try:
            time = int(args[0])
        except Exception:
            print(CGREEN+'Not a valid time value, it must be a int.'+CRESET)
            return
        try:
            delta = float(args[1])
        except Exception:
            print(CGREEN+'Not a valid delta value, it must be a float.'+CRESET)
            return
        # Making the robot reach the goal
        self._move_robot(time, delta)
        
        
    def _random_controller(self):
        '''
        This method is executed only if the controller is executed in RANDOM mode.
        It makes the robot move with the related method.
        '''
        # Making the robot reach the goal
        time = random.randrange(5, 10)
        delta = random.uniform(-0.2, 0,2)
        self._move_robot(time, delta)


    def _move_robot(self, time, goal_delta):
        '''
        Args:
            time (int) : The number of seconds that the robot takes to move.
            goal_delta (float) : The randomization of the path goal position.
        
        This method makes the robot wait for the specified number of seconds and then
        creates the ActionServer result by setting the robot position near the path
        goal position randomized with the specified amount.
        '''
        # Waiting for the specified number of seconds
        rospy.loginfo(CYELLOW+'Moving the robot for '+str(time)+' seconds.'+CRESET)
        rospy.sleep(time)
        # Filling and setting the result
        result = FollowPathResult()
        pos = self._goal.path[len(self._goal.path)-1]
        deltax = random.uniform(-goal_delta, goal_delta)
        deltay = random.uniform(-goal_delta, goal_delta)
        deltaz = random.uniform(-goal_delta, goal_delta)
        result.position = Point(pos.x+deltax, pos.y+deltay, pos.z+deltaz)
        self._set_action_result(result)
        rospy.loginfo(CYELLOW+'Completed, reached position is: \n'+str(result.position)+'.'+CRESET)
    


if __name__ == "__main__" :
    motion_controller = MotionController()
    motion_controller.start()
    rospy.spin()

