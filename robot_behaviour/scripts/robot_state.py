#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random
# Importing THREADING library to make system multithreaded
import threading
# Imporing ROS library for python
import rospy
import actionlib
from std_msgs.msg import UInt8
from robot_state_msgs.msg import MoveBetweenRoomsAction, MoveBetweenRoomsResult

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CVIOLET = '\33[35m'
CRESET  = '\033[0m'


class RobotState(object):
    '''
    |  This class represent the state of the a robot. 
    |  It is a collection of "state controllers": each one is a module that controls only 
    |  a dedicated section of the state of the robot. Therefore, a more complex robot state
    |  can be obtained by simply creating and adding more controllers.
    |  It also provides some functionalities to simplify the concurrent interaction of the 
    |  controllers with with the user.
    '''
    def __init__(self, state_controllers):
        '''
        Args:
            state_controllers (list) : The list of StateControllers for this state.
        '''
        # Initializing a ROS node which represents the robot state
        rospy.init_node('robot_state', log_level=rospy.INFO)
        # Storing all the state controllers
        self._state_controllers = state_controllers
        # The possible commands that the user can prompt
        self._registered_commands = {}


    def start(self) :
        '''
        |  This method is used to start the RobotState :
        |  1. Shows to the user all the possible commands.
        |  2. Starts all the StateControllers with the related method.
        |  3. Starts to listen for user commands.
        '''
        # First, explaining all the commands to the user
        for state_controller in self._state_controllers:
            print()
            state_controller.explain_commands()    
        # Starting all the state controllers
        print()
        for state_controller in self._state_controllers:
            # Defining the lambda for the controller start function
            thread_func = lambda : state_controller.start(self)
            thread = threading.Thread(target=thread_func)
            thread.start()
        # Starting to listen for user input
        self._listen_for_user_commands()


    def register_command(self, command, callback):
        '''
        Args:
            command (list) : The list of args that compose the command.
            callback (function) : The callback to invoke for the command.

        |  This is a helper method which can be used by StateControllers to
        |  register commands. The commands are deconstructed by arguments and
        |  stored in a tree structure with the related callbacks.
        '''
        # This source will contain at each iteration all the args that can
        # be appended after the current argument. Initially, there are all
        # the possibile commands.
        arg_source = self._registered_commands
        for arg in command :
            # Appending argument one next to each other
            if not arg in arg_source :
                arg_source[arg] = {}
            arg_source = arg_source[arg]
        # Appending the callback for later usage
        arg_source['_callback'] = callback

    
    def _listen_for_user_commands(self):
        '''
        |  This method continusly wait for the user input in the console.
        |  Once a valid command is received, the related callback is called.
        |  If the command is not valid, the user is informed.
        '''
        while True:
            # Parsing the command from string
            command = list(filter(None, input().split(' ')))
            # Stop listenin to commands when the user request it
            if len(command) == 1 and command[0] == 'exit':
                break
            # Decoding the command to retrieve the callback
            arg_source = self._registered_commands
            parameters = []
            for i in range(len(command)) :
                current_arg = command[i]
                if not current_arg in arg_source :
                    parameters = command[i:]
                    break
                arg_source = arg_source[current_arg]
            # Check if the callback actually exists
            if not '_callback' in arg_source:
                print('Incomplete or wrong command')
                continue
            # Invoking the callback with the last argument
            arg_source['_callback'](parameters)
                


class BatteryStateController(object):
    '''
    Publishes to:
        /battery_level (UInt8)

    |  This controller handles the battery level of the robot.
    |  It has both MANUAL and RANDOM execution modes.
    '''
    def __init__(self):
        '''
        |  This is the constructor method for the BatteryStateController class.
        |  It gets the required parameters from the rospy.
        '''
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/battery_controller_execution_mode', 'MANUAL')
        self._battery_level = rospy.get_param('/battery_intial_value', 100)
    
    
    def explain_commands(self):
        '''
        |  This method is used to print in console the available commands in console.
        |  Actually, the commands are shown to the user only if the controller is set
        |  in MANUAL execution mode, becuase in AUTOMAIC mode the commands are not used.
        '''
        rospy.loginfo(CGREEN+'Battery controller is started in '+self._execution_mode+' mode.'+CRESET)
        # Informing user how to control the battery
        if self._execution_mode == 'MANUAL' :
            print('You can interact with the battery controller with the commands:')
            print('- '+CGREEN+'battery offset <offset>'+CRESET+' :')
            print('\t 1. <offset> (int) : the offset to add to the current battery level.')
            print('- '+CGREEN+'battery set <value>'+CRESET+' :')
            print('\t 1. <value> (int) : the value to set to the battery level.')
    

    def start(self, robot_state):
        '''
        Args:
            robot_state (RobotState) : The RobotState this controller belongs to.
        
        |  This method starts the controller in the specified execution mode.
        |  Also, creates a Publisher to the /battery_level topic.
        '''
        # Storing the robot state to retrieve the commands
        self._robot_state = robot_state
        # Actually starting the controller in the specified mode
        target_controller=self._manual_battery_controller
        if self._execution_mode == 'MANUAL' :
            target_controller=self._manual_battery_controller
        elif self._execution_mode == 'RANDOM' :
            target_controller=self._random_battery_controller
        else :
            rospy.logerr('Invalid execution mode ('+self._execution_mode+').')
        # Defining a publisher to notify for when the battery is low
        self._battery_level_pub = rospy.Publisher('battery_level', UInt8, queue_size=1, latch=True)
        # Actually starting the controller
        target_controller()
    

    def _manual_battery_controller(self):
        '''
        |  This method is executed only if the controller is execute in MANUAL mode.
        |  It is used to register the commands that are necessary to for the 
        |  communication between the user and this controller.
        '''
        # Publishing the first battery level
        self._change_battery_level(0)
        # The callback for the offset command
        def _offset_callback(args) :
            try :
                self._change_battery_level(int(args[0]))
            except ValueError :
                print('Not a valid offset value, it must be a int.')
        # The callback for the set command
        def _set_callback(args) :
            try :
                self._change_battery_level(-self._battery_level + int(args[0]))
            except ValueError :
                print('Not a valid set value, it must be a int.')
        # Registering commands with the related callbacks
        self._robot_state.register_command(['battery','offset'], _offset_callback)
        self._robot_state.register_command(['battery','set'], _set_callback)
    

    def _random_battery_controller(self):
        '''
        |  This method is executed only if the controller is execute in AUTOMATIC mode.
        |  This method controls the battery level randomly: in a loop, waits
        |  some time and then the level is switched between 0 and 100.
        '''
        # Publishing the first battery level
        self._change_battery_level(0)
        # Looping to simulate time and decrease battery level
        while not rospy.is_shutdown():
            # Waiting for a random amount of time
            rospy.sleep(random.randrange(10, 15))
            # Switching battery state
            if self._battery_level > 0 :
                self._change_battery_level(-100)
            else :
                self._change_battery_level(+100)    
            

    def _change_battery_level(self, offset):
        '''
        Args:
            offset (int) : The offset to add to the current battery level.
        
        |  This method adds the offset to the battery level and clamps the
        |  value of the battery level in order to keep it between 0 and 100.
        |  Then publishes the new value of the battery level in the topic.
        '''
        # Changing the battery level with the given offset
        self._battery_level = self._battery_level + offset
        self._battery_level = min(max(0, self._battery_level), 100)
        # Publishing the new battery level
        self._battery_level_pub.publish(UInt8(self._battery_level))
        rospy.loginfo(CYELLOW+'Battery level is at '+str(self._battery_level)+'%'+CRESET)



class MoveStateController(object):
    '''
    Action Server:
        /robot_move - MoveBetweenRoomsAction

    |  This controller handles the movement of the robot.
    |  It has both MANUAL and RANDOM execution modes.
    '''
    def __init__(self):
        '''
        |  This is the constructor method for the MoveStateController class.
        |  It gets the required parameters from the rospy and initializes some variables.
        '''
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/move_controller_execution_mode', 'MANUAL')
        # Some variables for handling the server
        self._goal_available = False
        self._result_set_event = threading.Event()
        self._result = None
    
    
    def explain_commands(self):
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
        
        
    def start(self, robot_state):
        '''
        Args:
            robot_state (RobotState) : The RobotState this controller belongs to.
        
        |  This method starts the controller in the specified execution mode.
        |  Also, creates and starts an ActionServer with name /robot_move .
        '''
        # Storing the robot state to retrieve the commands
        self._robot_state = robot_state
        # Actually starting the controller in the specified mode
        target_controller=self._manual_move_controller
        if self._execution_mode == 'MANUAL' :
            target_controller=self._manual_move_controller
        elif self._execution_mode == 'RANDOM' :
            target_controller=None
        else :
            rospy.logerr('Invalid execution mode ('+self._execution_mode+').')
            
        # Defining an action server to simulate the robot moving
        self._move_srv = actionlib.SimpleActionServer('/robot_move', 
            MoveBetweenRoomsAction, execute_cb=self._move_action_callback, auto_start=False)
        # Starting the action server
        self._move_srv.start()
        
        # Actually starting the controller
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
        # The callback for the command
        def _move_callback(args):
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
        # Registering the command with the related callback
        self._robot_state.register_command(['move'], _move_callback)


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
        result = None
        if success :
            # In this simple case the path is simply the two rooms
            result = MoveBetweenRoomsResult()
            result.rooms = [self._goal.current_room, self._goal.next_room]
        # Loggin final result
        result = 'SUCCESS' if success else 'FAIL'
        rospy.loginfo(CYELLOW+'Movement complete with '+result+' result.'+CRESET)
        return result
        
        

if __name__ == '__main__' :
    # Initializing BatteryStateController
    state_controllers = [
        BatteryStateController(),
        MoveStateController()
    ]

    # Creating a RobotState to handle robot state
    robot_state = RobotState(state_controllers)
    robot_state.start()
    
