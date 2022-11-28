#!/usr/bin/env python

# Importing AST library for safely evaluating strings
import ast
# The RANDOM library is necessary for debugging
import random

# Importing ROS library for python
import rospy

# Importing SMACH to create FINITE STATE MACHINES
from smach import StateMachine, State

# Importing HELPER to abstract interaction with ARMOR
from robot_behavior.robot_behavior_helper import RobotBehaviorHelper


CRED    = '\33[31m'
CGREEN  = '\33[32m'
CYELLOW = '\33[33m'
CRESET  = '\033[0m'

class Initialization(State) :
    '''
    Args:
        helper (RobotBehaviorHelper) : An instance of the RobotBehaviorHelper.

    This class is an instance of State used for the initialization
    of the parameters which define the behavior of the robot.
    Because of this functionality, it should be executed only once. 
    '''
    def __init__(self, helper):
        State.__init__(self, outcomes=['initialized'])
        # Storing th helper for possible usage
        self._helper = helper
    
    def execute(self, userdata):
        '''
        Args:
            userdata (object) : The data of this State.
        Returns:
            (string) : The outcome 'initialized' when the initialization is complete.
            
        |  This state initializes the robot behavior by:
        |  1. Waits until the ontology is fully loaded.
        '''
        # Waiting until the ontology is loaded into ARMOR
        rospy.loginfo(CYELLOW+'Waiting for ontology to be fully loaded.'+CRESET)
        self._helper.wait_for_ontology()
        rospy.loginfo(CYELLOW+'Received ontology.'+CRESET)
        # Updates Robot urgency threshold
        threshold = self._helper.urgency_threshold
        self._helper.update_robot_urgency_threshold(threshold)
        # Returns the outcome
        return 'initialized'


class ChooseTaskForCurrentRoom(State) :
    '''
    Args:
        helper (RobotBehaviorHelper) : An instance of the RobotBehaviorHelper.

    This class is an instance of State used for choosing the most 
    suitable task to be executed by the robot. In particular, the
    task is chosen consider some parameters of the robot and the
    capabilities of the room.
    '''
    def __init__(self, helper):
        State.__init__(self, outcomes=['recharge', 'explore'])
        # Storing the helper for possible usage
        self._helper = helper
    
    def execute(self, userdata):
        '''
        Args:
            userdata (object) : The data of this State.
        Returns:
            (string) : 
                - The outcome 'recharge' when the battery needs recharge.
                - The outcome 'explore' when it is possible to explore.
            
        |  This states choosing the next task to be executed:
        |  1. If the battery is low and there is a recharge station in the 
              room, then the robot must recharge.
        |  2. Otherwise the robot must explore the room.
        '''    
        # Check if robot needs recharging and this is a recharge room
        current_room = self._helper.retrieve_current_room()
        is_room_for_charge = current_room in self._helper.recharge_rooms
        if _should_robot_recharge(self._helper) and is_room_for_charge:
            return 'recharge'
        # Otherwise another task is performed (exploration)
        return 'explore'

        
class ExploreTask(State):
    '''
    Args:
        helper (RobotBehaviorHelper) : An instance of the RobotBehaviorHelper.

    This class is an instance of State which simulates the robot
    exploring the room it is currently in.
    '''
    def __init__(self, helper):
        State.__init__(self, outcomes=['explored'])
        # Storing the helper for possible usage
        self._helper = helper
    
    def execute(self, userdata):
        '''
        Args:
            userdata (object) : The data of this State.
        Returns:
            (string) : The outcome 'explored' when exploration is complete.
            
        The robot explores the room for the amount of seconds that
        the user specified in the launch file.
        '''    
        # Simulating the robot exploring the room
        rospy.loginfo(CYELLOW+'Exploring room (%i seconds).'+CRESET, self._helper.exploration_seconds)
        rospy.sleep(self._helper.exploration_seconds)
        rospy.loginfo(CYELLOW+'Finished exploring room.'+CRESET)
        return 'explored'


class RechargeTask(State):
    '''
    Args:
        helper (RobotBehaviorHelper) : An instance of the RobotBehaviorHelper.
        
    This class is an instance of State which simulates the robot
    recharging into the designated recharge room.
    '''
    def __init__(self, helper):
        State.__init__(self, outcomes=['recharged'])
        # Storing the helper for possible usage
        self._helper = helper
        
    def execute(self, userdata):
        '''
        Args:
            userdata (object) : The data of this State.
        Returns:
            (string) : The outcome 'recharged' when the recharge is complete.
            
        This method makes the robot wait until the battery is charged
        enough. The threshold is parameter defined by the user.
        '''    
        # Simulating the robot recharging
        rospy.loginfo(CYELLOW+'Starting to recharge the battery.'+CRESET)
        while not _should_robot_stop_recharge(self._helper) :
            rospy.sleep(2)
        bl = self._helper.get_battery_level()
        rospy.loginfo(CYELLOW+'Completed recharge with battery at %s%%.'+CRESET, bl)
        return 'recharged'


class ChooseNextRoom(State) :
    '''
    Args:
        helper (RobotBehaviorHelper) : An instance of the RobotBehaviorHelper.
        
    This class is an instance of State which decides the next room
    the robot should visit based on the following policy: if the
    robot has not enough battery to reach another room, a room which
    contains a recharge station is chosen. Otherwise, the reachable
    room with the highest priority is chosen.
    '''
    def __init__(self, helper):
        State.__init__(self, outcomes=['chosen'], output_keys=['next_room'])
        # Storing the helper for possible usage
        self._helper = helper
        
    def execute(self, userdata):
        '''
        Args:
            userdata (object) : The data of this State.
        Returns:
            (string) : The outcome 'chosen' when a room has been selected.
            
        If the battery of robot is not enough to reach another room,
        makes the robot go into the room with the recharging station.
        Otherwise, makes the robot go into another room depending on
        the rooms priorities.
        '''    
        # The robot time must be updated before choosing another room
        self._helper.update_robot_time()
        # Choosing the next room to visit
        if _should_robot_recharge(self._helper) :
            # If the battery level is low, it must be charged
            room = _rand_in_list(self._helper.recharge_rooms)
            bl = self._helper.get_battery_level()
            rospy.loginfo(CYELLOW+'Going to recharge since battery level is low (%s%%).'+CRESET, bl)
        else :
            # If the battery is not low, choose a room based on priority
            room, clss = self.choose_next_prioritized_room()            
            rospy.loginfo(CYELLOW+'Choosing a prioritized room of type %s.'+CRESET, clss)
        # Loggin chosen next room
        rospy.loginfo(CYELLOW+'The next room is: %s .'+CRESET, room)
        userdata.next_room = room
        return 'chosen'
    
    def choose_next_prioritized_room(self):
        '''
        Returns:
            (string, string) : A tuple containing the reachable room with the 
            highest priority and its related class name.
        
        The rooms priorities are decided by user with the dedicated parameter.
        This method loops over all the priorities in order and checks if
        there is a reachable room of the given priority.
        If no room is found, returns any reachable room.
        '''
        # Retrieve the list of all the reachable rooms from the robot position
        reachable_rooms = self._helper.retrieve_reachable_rooms()
        
        for room_clss in self._helper.rooms_priority :
            # Obtaining the reachable rooms for the given priority class
            priority_rooms = self._helper.retrieve_rooms_of_class(room_clss)
            rooms = list(set(reachable_rooms) & set(priority_rooms))
            if rooms :
                return [_rand_in_list(rooms), room_clss]
        
        # Otherwise, go into any reachable room
        return  [_rand_in_list(reachable_rooms), 'ROOM']


class MoveToNextRoom(State) :
    '''
    Args:
        helper (RobotBehaviorHelper) : An instance of the RobotBehaviorHelper.
        
    This class is an instance of State which makes the robot move
    to the selected room.
    '''
    def __init__(self, helper):
        State.__init__(self, outcomes=['moved'], input_keys=['next_room'])
        # Storing the helper for possible usage
        self._helper = helper
    
    def execute(self, userdata):
        '''
        Args:
            userdata (object) : The data of this State.
        Returns:
            (string) : The outcome 'moved' when the robot moved to the selected room.
        
        Makes the robot move from the current room to the selected room.
        '''
        # Updating the time the robot visited the room
        self._helper.update_room_visited_time_before_exiting()
        # Taking the chosen room
        room = userdata.next_room
        # Moving to next room
        rospy.loginfo(CYELLOW+'Starting to compute a path for room %s .'+CRESET, room)
        path = self._helper.compute_path_to_room(room)
        rospy.loginfo(CYELLOW+'Received the path for room %s .'+CRESET, room)
        rospy.loginfo(CYELLOW+'Starting to move on the computed path.'+CRESET)
        self._helper.follow_path_for_room(path, room)
        rospy.loginfo(CYELLOW+'Completed movement, the current room is %s .'+CRESET, room)
        return 'moved'



class RobotSurveillance(object):
    '''
    ROS Parameters:
        - /urgency_threshold (int) : The urgency threshold time in seconds for the robot policy.
        - /battery_require_recharge (int) : The battery level under which it is necessary to start recharging.
        - /battery_stop_recharge (int) : The battery level over which it is possible to stop recharging.
        - /recharge_rooms (list) : The list of rooms containing a recharge station.
        - /rooms_priority (list) : The list of room classes in order of priority.
        - /exploration_seconds (int) : The time in seconds the robot takes to explore a room.
        
    This class generates the robot behavior by creating a state machine.
    '''
    def __init__(self):
        # Initializing a ROS node which represents the robot behavior
        rospy.init_node('robot_behavior', log_level=rospy.INFO)
        # Creating a instance of the helper
        self._helper = RobotBehaviorHelper()
        # Setting some parameters to personalize behavior
        self._helper.urgency_threshold = rospy.get_param('/urgency_threshold')
        self._helper.battery_require_recharge = rospy.get_param('/battery_require_recharge', 20)
        self._helper.battery_stop_recharge = rospy.get_param('/battery_stop_recharge', 100)
        self._helper.recharge_rooms = ast.literal_eval(rospy.get_param('/recharge_rooms'))
        self._helper.rooms_priority = ast.literal_eval(rospy.get_param('/rooms_priority'))
        self._helper.exploration_seconds = rospy.get_param('/exploration_seconds', 10)

    def generate_behavior(self):
        '''
        Returns:
            (StateMachine) : The state machine representing the behavior.
            
        In this method the actual behavior of the robot is created: in this
        context the robot is initialized and then explores the environment by
        normally moving in corridors but moving into a room if it didn't explore
        it for some time. When the battery of the robot is low, it goes into
        a room containing a recharge station and charges until the battery
        level is enough to start moving again.
        '''
        # A state machine that describes the behavior of the robot
        behavior_sm = StateMachine(outcomes=[])
        with behavior_sm :
            # A state for the initialization of the robot
            StateMachine.add('INITIALIZATION', Initialization(self._helper),
                    transitions={'initialized':'PERFORM_ROOM_TASKS'})
            
            # A state machine for performing a task in a room
            task_sm = StateMachine(outcomes=['task_completed'])
            with task_sm :
                # A task for deciding which is the task to perform in the room
                StateMachine.add('CHOOSE_ROOM_TASK', ChooseTaskForCurrentRoom(self._helper),
                            transitions={'explore':'EXPLORE_TASK',
                                        'recharge':'RECHARGE_TASK'})
                # This task makes the robot explore the current room
                StateMachine.add('EXPLORE_TASK', ExploreTask(self._helper),
                            transitions={'explored':'task_completed'})
                # This task makes the robot recharge in the current room
                StateMachine.add('RECHARGE_TASK', RechargeTask(self._helper),
                            transitions={'recharged':'task_completed'})
                            
            # A state for performing tasks in the chosen room
            StateMachine.add('PERFORM_ROOM_TASKS', task_sm,
                    transitions={'task_completed':'CHOOSE_NEXT_ROOM'})
            # A state for choosing the next room to reach
            StateMachine.add('CHOOSE_NEXT_ROOM', ChooseNextRoom(self._helper),
                    transitions={'chosen':'MOVE_TO_NEXT_ROOM'},
                    remapping={'next_room':'next_room'})
            # A state for reaching the next chosen room
            StateMachine.add('MOVE_TO_NEXT_ROOM', MoveToNextRoom(self._helper),
                    transitions={'moved':'PERFORM_ROOM_TASKS'},
                    remapping={'next_room':'next_room'})
        
        return behavior_sm
        

def _rand_in_list(lst):
    '''
    Args:
        lst (list) : A list of objects.
    Returns:
        (object): A random object inside the list.
    '''
    return lst[random.randrange(len(lst))]


def _should_robot_recharge(helper):
    '''
    Returns:
        (bool) : If the robot should recharge.
    '''
    return helper.get_battery_level() <= helper.battery_require_recharge


def _should_robot_stop_recharge(helper):
    '''
    Returns:
        (bool) : If the robot should stop recharging.
    '''
    return helper.get_battery_level() >= helper.battery_stop_recharge


if __name__ == '__main__':
    # Creating an instance of the RobotBehavior
    robot_behavior = RobotSurveillance()
    # Actually generating the behavior of the robot
    behavior_sm = robot_behavior.generate_behavior()
    # Executing the behavior of the robot
    outcome = behavior_sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()



