#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random
# Importing THREADING library to make system multithreaded
import threading
# Imporing ROS library for python
import rospy
from std_msgs.msg import UInt8


class RobotState(object):
    '''
    |  This class represent the state of the a robot: it is based on the
    |  concept of "state controllers" each one should be considered a module
    |  that controls only a part of the state. It also provides some
    |  functionalities to simplify the interaction of the controllers with
    |  with the user.
    '''
    def __init__(self, state_controllers):
        '''
        Args:
            state_controllers: The list of StateControllers for this state.
        '''
        # Initializing a ROS node which represents the robot state
        rospy.init_node('robot_state', log_level=rospy.INFO)
        # Storing all the state controllers
        self._state_controllers = state_controllers
        # The possible commands that the user can prompt
        self._registered_commands = {}


    def start(self) :
        '''
        |  1. Starts all the StateControllers with the related method.
        |  2. Starts to listen for user commands.
        '''
        # Starting all the state controllers
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
            command: The command to register.
            callback: The callback to invoke for the command.

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
            # Decoding the command to retrieve the callback
            arg_source = self._registered_commands
            for arg in command[:-1] :
                if not arg in arg_source :
                    print('Invalid argument ('+arg+').')
                    break
                arg_source = arg_source[arg]
            else :
                # Check if the callback actually exists
                if not '_callback' in arg_source:
                    print('Incomplete command')
                    break
                # Invoking the callback with the last argument
                arg_source['_callback'](command[-1])
                


class BatteryStateController(object):
    '''
    |  This class handles the battery level state of the robot.
    |  It has MANUAL and RANDOM execution modes.
    '''
    def __init__(self):
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/battery_controller_execution_mode')
        self._battery_level = rospy.get_param('/battery_intial_value')
        # Defining a publisher to notify for when the battery is low
        self._battery_level_pub = rospy.Publisher('battery_level', UInt8, queue_size=1, latch=True)
    

    def start(self, robot_state):
        '''
        Args:
            robot_state: The RobotState this controller belongs to.
        
        |  This method starts the actual controller method into a different thread
        |  so that other controlles can be executed concurrently.
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
        # Actually starting the controller
        target_controller()
        rospy.loginfo('Battery controller is started in '+self._execution_mode+' mode.')
    

    def _manual_battery_controller(self):
        '''
        |  This method registers the commands that are necessary to communica with
        |  the user to the robot state. 
        '''
        # Publishing the first battery level
        self._change_battery_level(0)
        # Informing user how to control the batter
        print('You can interact with the battery controller by typing:')
        print('1. battery_controller offset <offset> : offsets the value of the battery level.')
        print('2. battery_controller set <value> : sets the value of the battery level.')

        def _offset_callback(offset) :
            try :
                self._change_battery_level(int(offset))
            except ValueError :
                print('Not a valid offset for the offset.')
        
        self._robot_state.register_command(['battery_level','offset'], _offset_callback)

        def _set_callback(level) :
            try :
                self._change_battery_level(-self._battery_level + int(level))
            except ValueError :
                print('Not a valid offset for the battery level.')
        
        self._robot_state.register_command(['battery_level','set'], _set_callback)
    

    def _random_battery_controller(self):
        '''
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
            offset: The offset to add to the current battery level.
        
        |  This method adds the offset to the battery level and clamps the
        |  value of the battery level in order to keep it between 0 and 100.
        |  Then publishes the new value of the battery level in the topic.
        '''
        # Changing the battery level with the given offset
        self._battery_level = self._battery_level + offset
        self._battery_level = min(max(0, self._battery_level), 100)
        # Publishing the new battery level
        self._battery_level_pub.publish(UInt8(self._battery_level))
        rospy.loginfo('Battery level is at '+str(self._battery_level)+'%')



class MoveStateController(object):
    def __init__(self):
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/move_controller_execution_mode')
        # Defining an action server to simulate the robot moving
        self._move_pub = rospy.Publisher('battery_level', UInt8, queue_size=1, latch=True)
    
    
    def start(self, robot_state):
        # Storing the robot state to retrieve the commands
        self._robot_state = robot_state
        # Actually starting the controller in the specified mode
        target_controller=self._manual_move_controller
        if self._execution_mode == 'MANUAL' :
            target_controller=self._manual_move_controller
        elif self._execution_mode == 'RANDOM' :
            target_controller=self._random_move_controller
        else :
            rospy.logerr('Invalid execution mode ('+self._execution_mode+').')
        # Actually starting the controller
        target_controller()
        rospy.loginfo('Move controller is started in '+self._execution_mode+' mode.')
    

    def _manual_move_controller(self):
        # Informing user how to control the batter
        print('You can interact with the move controller by typing:')
        print('1. move_controller move [<time>, <result>] : which makes the movement take <time>' +
                  'seconds to complete and at the end, the <result> boolean value will be the result'+
                  'of the movement (if the target has been correctly reached).')
        
        def _move_callback(args):
            # Parsing the arguments
            time = args[0]
            result = args[1]
            # Simulate the passing of time
            rospy.sleep(time)
            # Sending the result
            # TODO
        
        self._robot_state.register_command(['move_controller', 'move'], self._move_callback)
        

if __name__ == '__main__' :
    # Initializing BatteryStateController
    state_controllers = [
        BatteryStateController(),
        MoveStateController()
    ]

    # Creating a RobotState to handle robot state
    robot_state = RobotState(state_controllers)
    robot_state.start()
    # Spinning to prevent exiting
    rospy.spin()
