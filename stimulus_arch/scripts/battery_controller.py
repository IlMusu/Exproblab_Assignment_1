#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random

# Importing ROS library for python
import rospy
from std_msgs.msg import UInt8

# Importing the helper for handling user commands
from stimulus_arch.command_helper import CommandHelper
from stimulus_arch.stimulus_controller import StimulusController

CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class BatteryController(StimulusController):
    '''
    Publishes to:
        - /battery_level (UInt8)
    ROS Parameters :
        - /battery_initial_value (int) : The initial value for the battery level.

    This class is a controller that handles the battery level of the robot.
    '''
    
    def __init__(self):
        '''
        |  This is the constructor method for the BatteryController class.
        |  1. Initializes the /battery_controller node.
        |  2. Gets the required parameters from the rospy.
        '''
        StimulusController.__init__(self)
        # Initializing the ROS node
        rospy.init_node('battery_controller', log_level=rospy.INFO)
        # Initializing the battery level to the max
        self._battery_level = rospy.get_param('/battery_initial_value', 100)
    
    
    def _explain_commands(self):
        '''
        This method is used to print in console the available commands in console.
        Actually, the commands are shown to the user only if the controller is set
        in MANUAL execution mode, because in RANDOM mode the commands are not used.
        '''
        rospy.loginfo(CGREEN+'Battery controller is started in '+self._execution_mode+' mode.'+CRESET)
        # Informing user how to control the battery
        if self._execution_mode == 'MANUAL' :
            print('You can interact with the battery controller with the commands:')
            print('- '+CGREEN+'battery offset <offset>'+CRESET+' :')
            print('\t 1. <offset> (int) : the offset to add to the current battery level.')
            print('- '+CGREEN+'battery set <value>'+CRESET+' :')
            print('\t 1. <value> (int) : the value to set to the battery level.')
            print()
    

    def start(self):
        '''
        This method creates a Publisher for the /battery_level topic.
        And then starts the StimulusController.
        '''
        # Creating a publisher for the /battery_level topic
        self._battery_level_pub = rospy.Publisher('battery_level', UInt8, queue_size=1, latch=True)
        # Actually starting the controller
        StimulusController.start(self)
    

    def _manual_controller(self):
        '''
        This method is executed only if the controller is executed in MANUAL mode.
        It is used to register the commands that are necessary to for the communication 
        between the user and this controller.
        '''
        # Publishing the first battery level
        self._offset_battery_level(0)
        # Registering the commands
        self._commands = CommandHelper()
        # Registering commands with the related callbacks
        self._commands.register_command(['battery','offset'], self._offset_command)
        self._commands.register_command(['battery','set'], self._set_command)
        # Handling commands inserted by the user
        self._commands.listen_for_user_commands('exit')
    

    def _random_controller(self):
        '''
        This method is executed only if the controller is executed in RANDOM mode.
        This method controls the battery level randomly: in a loop, waits some time 
        and then the level is switched between 0 and 100.
        '''
        # Publishing the first battery level
        self._offset_battery_level(0)
        # Looping to simulate time and decrease battery level
        while not rospy.is_shutdown():
            # Waiting for a random amount of time
            rospy.sleep(random.randrange(10, 15))
            # Switching battery state
            if self._battery_level > 0 :
                self._offset_battery_level(-100)
            else :
                self._offset_battery_level(+100)    
            
        
    def _offset_command(self, args) :
        '''
        Args:
            args (list) : The list of arguments for the command.

        This method is the callback for the "battery offset" command, it modifies
        the battery level value with the specified offset.
        '''
        try :
            self._offset_battery_level(int(args[0]))
        except ValueError :
            print(CGREEN+'Not a valid offset value, it must be a int.'+CRESET)
            
            
    def _set_command(self, args) :
        '''
        Args:
            args (list) : The list of arguments for the command.

        This method is the callback for the "battery set" command, it sets the
        battery level value with the specified value.
        '''
        try :
            self._offset_battery_level(-self._battery_level + int(args[0]))
        except ValueError :
            print(+CGREEN+'Not a valid set value, it must be a int.'+CRESET)
    
    
    def _offset_battery_level(self, offset):
        '''
        Args:
            offset (int) : The offset to add to the current battery level.
        
        This method adds the offset to the battery level and clamps the
        value of the battery level in order to keep it between 0 and 100.
        Then publishes the new value of the battery level in the topic.
        '''
        # Changing the battery level with the given offset
        self._battery_level = self._battery_level + offset
        self._battery_level = min(max(0, self._battery_level), 100)
        # Publishing the new battery level
        self._battery_level_pub.publish(UInt8(self._battery_level))
        rospy.loginfo(CYELLOW+'Battery level is at '+str(self._battery_level)+'%'+CRESET)
        


if __name__ == "__main__" :
    battery_controller = BatteryController()
    battery_controller.start()
    rospy.spin()


    
