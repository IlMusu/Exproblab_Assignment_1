#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random
# Importing THREADING library to make system multithreaded
import threading
# Imporing ROS library for python
import rospy
from std_msgs.msg import UInt8, Bool

class RobotState(object):
    def __init__(self):
        # Initializing a ROS node which represents the robot state
        rospy.init_node('robot_state', log_level=rospy.INFO)
        # Initializing BatteryController
        self._battery_controller = BatteryController()

    def start(self) :
        # Starting the BatteryController
        self._battery_controller.start(self._battery_controller_em)
    

class BatteryController(object):
    def __init__(self):
        # Initializing the battery level to the max
        self._execution_mode = rospy.get_param('/battery_controller_execution_mode')
        self._battery_level = rospy.get_param('/battery_intial_value')
        # Defining a publisher to notify for when the battery is low
        self._battery_level_pub = rospy.Publisher('battery_level', UInt8, latch=True)
        # Defining a listener for the recharge
        self._battery_recharging_sub = rospy.Subscriber('battery_recharging', Bool, latch=True)
    

    def start(self):
        match self._execution_mode :
            case _, 'MANUAL' :
                target=self._manual_battery_controller
            case 'RANDOM' :
                target=self._random_battery_controller

        rospy.loginfo('Battery controller is started in '+execution_mode+' mode')
        thread = threading.Thread(target=target)
        thread.start()
    

    def _manual_battery_controller(self):
        # Publishing the first battery level
        self._change_battery_level(0)
        # Informing user how to control the batter
        print('You can change the battery level by typing the offset value (int).')
        print('A positive value to incrase the battery level and a negative value to decrease it.')
        while not rospy.is_shutdown():
            try :
                battery_offset = int(input())
            except ValueError :
                print('The inserted value is not a valid offset.')
            self._change_battery_level(battery_offset)
    

    def _random_battery_controller(self):
        # Publishing the first battery level
        self._publish_battery_level()
        # Looping to simulate time and decrease battery level
        while not rospy.is_shutdown():
            # Waiting for a random amount of time
            sleep(random.randrange(15, 25))
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
        self._battery_low_pub.publish(UInt8(self._battery_level))
        rospy.loginfo('Battery level is at '+str(self._battery_level)+'%')
        

if __name__ == "__main__" :
    # Creating a RobotState to handle robot state
    robot_state = RobotState()
    robot_state.start()
    # Spinning to prevent exiting
    rospy.spin()