#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random
# Importing THREADING library to make system multithreaded
import threading
# Importing the helper for handling user commands
from command_helper import CommandHelper
# Imporing ROS library for python
import rospy
import actionlib
from robot_state_msgs.msg import MoveBetweenRoomsAction, MoveBetweenRoomsResult

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class MoveController(object):
    '''
    Action Server:
        /robot_move - MoveBetweenRoomsAction

    |  This controller handles the movement of the robot.
    |  It has both MANUAL and RANDOM execution modes.
    '''
    def __init__(self):
        '''
        |  This is the constructor method for the MoveController class.
        |  1. Initializes the /battery_controller ros node.
        |  2. Gets the required parameters from the rospy.
        |  3. Stars the /robot_move action server.
        '''
        # Initializing the ROS node
        rospy.init_node('move_controller', log_level=rospy.INFO)
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/execution_mode', 'MANUAL')
        # Some variables for handling the server
        self._goal = None
        self._result_set_event = threading.Event()
        self._result = None
    
    
    def _explain_commands(self):
        '''
        |  This method is used to print in console the available commands in console.
        |  Actually, the commands are shown to the user only if the controller is set
        |  in MANUAL execution mode, becuase in AUTOMAIC mode the commands are not used.
        '''
        rospy.loginfo(CGREEN+'Move controller is started in '+self._execution_mode+' mode.'+CRESET)
        # Informing user how to use the move controller
        if self._execution_mode == 'MANUAL' :
            print('You can interact with the battery controller with the commands:')
            print('- '+CGREEN+'move <time> <success>'+CRESET+' :')
            print('\t 1. <time> (int) : the time in seconds to perform the movement.')
            print('\t 2. <success> (true/false) : if the result will be successfull.')
        
        
    def start(self):
        '''
        |  This method creates an ActionServer for the /robot_move action and
        |  starts the controller in the specified execution mode.
        '''
        # Defining an action server to simulate the robot moving
        self._move_srv = actionlib.SimpleActionServer('/robot_move', MoveBetweenRoomsAction, 
                            execute_cb=self._move_action_callback, auto_start=False)
        self._move_srv.start()
        # Starting the controller in the specified mode
        target_controller=self._manual_move_controller
        if self._execution_mode == 'MANUAL' :
            target_controller=self._manual_move_controller
        elif self._execution_mode == 'RANDOM' :
            target_controller=None
        else :
            rospy.logerr('Invalid execution mode ('+self._execution_mode+').')
        
        # Actually starting the controller
        self._explain_commands()
        if not target_controller == None :
            target_controller()
    
    
    def _move_action_callback(self, goal):
        '''
        Args:
            goal (MoveBetweenRoomsGoal) : The requested goal.
        
        | This is the callback for the /robot_move ActionServer:
        | 1. If the execution mode is MANUAL, it waits for user input to set
        |    the result of the requested goal.
        | 2. If the execution mode is AUTOMATIC, the result is automatically
        |    computed by the controller.
        | The it sets the computed result to the ActionServer.
        '''
        # Storing the goal to be used later
        self._goal = goal
        # Informing the user that he has to insert the command to make the robot move
        if self._execution_mode == 'MANUAL' :
            print(CGREEN+'A movement request from room '+goal.current_room+' to room '+ 
                    goal.next_room+' is now available.'+CRESET)
            print(CGREEN+'The movement will not happen until you insert the related command.'+CRESET)
            # Waiting for the user to set the response
            self._result_set_event.clear()
            self._result_set_event.wait()
            self._result_set_event.clear()
        else :
            self._random_move_controller()
        # The result is now available
        if self._result == None :
            self._move_srv.set_aborted()
        else :
            self._move_srv.set_succeeded(self._result)
        # Resetting the state
        self._goal = None
        self._result = None
        
    
    def _manual_move_controller(self):
        '''
        |  This method is executed only if the controller is execute in MANUAL mode.
        |  It is used to register the commands that are necessary to for the 
        |  communication between the user and this controller.
        '''
        # Registering the command with the related callback
        self._commands = CommandHelper()
        self._commands.register_command(['move'], self._move_command)
        # Handling commands inserted by the user
        self._commands.listen_for_user_commands('exit')


    def _random_move_controller(self):
        '''
        |  This method is executed only if the controller is execute in AUTOMATIC mode.
        |  This method controls the movement of the robot randomly: the robot takes a
        |  random number of seconds to complete the movement and then the result of 
        |  the movement is also computed randomly.
        '''
        # Completing the command
        time = random.randrange(5, 10)
        success = random.choice([True, False])
        # Creating the result after the robot moved
        self._result = self._move_robot(time, success)
    

    def _move_command(self, args):
        # Check if the robot has actually requested a path
        if not self._goal :
            print("The robot did not yet request a movement!")
            return
        # Casting the first argument to int
        try :
            time = max(int(args[0]), 0)
        except ValueError :
            print('Not a valid time value, it must be a int.')
            return
        # Casting the second parameter to bool
        if args[1] != 'true' and args[1] != 'false' :
            print('Not a valid succeded value, it must either "true" or "false".')
            return
        # Creating the result after the robot moved
        self._result = self._move_robot(time, args[1] == 'true')
        self._result_set_event.set()
        

    def _move_robot(self, time, success):
        '''
        Args:
            time (int) : The number of seconds the robot takes to move.
            success (bool) : If the result of the movement is successful.
        Returns:
            (MoveBetweenRoomsResult) : The result for the requested goal.
        
        |  This method simulates the robot moving in the environment to reach
        |  a requested room. In order to do this, the robot takes some time
        |  to complete the movement and at then the movement may not be successful.
        |  In case the movement is not successful, it is as if the robot did
        |  not move from the initial position.
        '''
        # Loggin a info to the player
        rospy.loginfo(CYELLOW+'Moving for '+str(time)+' seconds.'+CRESET)
        # Simulate the passing of time
        rospy.sleep(time)
        # Creating the action result
        result = MoveBetweenRoomsResult()
        result.success = success
        result.rooms = [self._goal.current_room, self._goal.next_room]
        result.current_room = self._goal.next_room if success else self._goal.current_room
        # Loggin final result
        success_str = 'SUCCESS' if success else 'FAIL'
        rospy.loginfo(CYELLOW+'Movement complete with '+success_str+' result.'+CRESET)
        return result



if __name__ == "__main__" :
    move_controller = MoveController()
    move_controller.start()
