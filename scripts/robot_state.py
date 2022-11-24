#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random
# Importing THREADING library to make system multithreaded
import threading
# Imporing ROS library for python
import rospy
from std_msgs.msg import UInt8


class RobotState(object):
    def __init__(self, state_controllers):
        # Initializing a ROS node which represents the robot state
        rospy.init_node('robot_state', log_level=rospy.INFO)
        # Storing all the state controllers
        self._state_controllers = state_controllers
        # The possible commands that the user can prompt
        self._registered_commands = {}


    def start(self) :
        # Starting all the state controllers
        for state_controller in self._state_controllers:
            state_controller.start(self)
        # Starting to listen for user input
        self._listen_for_user_commands()


    def register_command(self, command, callback):
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
    def __init__(self):
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/battery_controller_execution_mode')
        self._battery_level = rospy.get_param('/battery_intial_value')
        # Defining a publisher to notify for when the battery is low
        self._battery_level_pub = rospy.Publisher('battery_level', UInt8, queue_size=1, latch=True)
    

    def start(self, robot_state):
        # Storing the robot state to retrieve the commands
        self._robot_state = robot_state
        # Actually starting the controller in the specified mode
        target=self._manual_battery_controller
        if self._execution_mode == 'MANUAL' :
            target=self._manual_battery_controller
        elif self._execution_mode == 'RANDOM' :
            target=self._random_battery_controller
        else :
            rospy.logerr('Invalid execution mode ('+self._execution_mode+').')

        rospy.loginfo('Battery controller is started in '+self._execution_mode+' mode.')
        thread = threading.Thread(target=target)
        thread.start()
    

    def _manual_battery_controller(self):
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
        # Changing the battery level with the given offset
        self._battery_level = self._battery_level + offset
        self._battery_level = min(max(0, self._battery_level), 100)
        # Publishing the new battery level
        self._battery_level_pub.publish(UInt8(self._battery_level))
        rospy.loginfo('Battery level is at '+str(self._battery_level)+'%')
        

if __name__ == '__main__' :
    # Initializing BatteryStateController
    state_controllers = [
        BatteryStateController()
    ]

    # Creating a RobotState to handle robot state
    robot_state = RobotState(state_controllers)
    robot_state.start()
    # Spinning to prevent exiting
    rospy.spin()
