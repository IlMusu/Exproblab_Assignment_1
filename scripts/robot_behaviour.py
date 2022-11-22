#!/usr/bin/env python

# Importing sys to handling correct exiting of the program
import sys
# The random library is necessary for debugging
import random

# Imporing the ROS library for python
import rospy
from rospy.exceptions import ROSInterruptException

# Importing SMACH to create FINITE STATE MACHINES
from smach import StateMachine, State

# Importing libraties to comunicate with ARMOR
from armor_helper import ArmorHelper
from armor_api.armor_client import ArmorClient

ROOM_EXPLORATION_TIME = 10
RECHARGE_ROOM = 'E'
RECHARGE_PER_SECOND = 10


class Initialization(State) :
	"""
    Args:
        helper : An instance of ArmorHelper in order to communicate with ARMOR.
        
    |  This class is an instance of State used for initializing the robot behaviour.
	|  Because of this, it should be executed only once. 
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['initialized'])
		# Storing th helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		"""
		Args:
			userdata : The data of this State.
		Returns:
			The outcome of this state which is always 'initialized'.
			
		|  This state initializes the ontology by:
		|  1. Loading the ontology into the ARMOR server.
		|  2. Setting the battery level of the robot the 100%.
		"""
		# Loading ontology into armor 
		self._helper.load_ontology()
		# Initializing battery level at the maximum
		self._helper.battery_level = 100
		return 'initialized'


class ChooseTaskForCurrentRoom(State) :
	"""
    Args:
        helper : An instance of ArmorHelper in order to communicate with ARMOR.
        
    |  This class is an instance of State used for choosing the 
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['recharge', 'explore'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):		
		# Check if robot needs recharging and this is a recharge room
		current_room = self._helper.retrieve_current_room()
		if self._helper.battery_level <= 0 and current_room == RECHARGE_ROOM:
			return 'recharge'
		
		# Otherwise another task is perfomed (exploration)
		return 'explore'

		
class ExploreTask(State):
	def __init__(self, helper):
		State.__init__(self, outcomes=['explored'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		# Simulating the robot exploring the room
		rospy.loginfo('Exploring room (%i seconds)', ROOM_EXPLORATION_TIME)
		#rospy.sleep(ROOM_EXPLORATION_TIME)
		input() # TODO remove this
		rospy.loginfo('Finished exploring room')
		# The robot moved in the room, it looses some battery
		self._helper.battery_level -= random.randrange(15, 35)
		self._helper.battery_level = max(self._helper.battery_level, 0)
		rospy.loginfo('Battery level is: '+str(self._helper.battery_level))
		return 'explored'


class RechargeTask(State):
	def __init__(self, helper):
		State.__init__(self, outcomes=['recharged'])
		# Storing the helper for possible usage
		self._helper = helper
		
	def execute(self, userdata):
		# Simulating the robot recharging
		rospy.loginfo('Starting recharge')
		while self._helper.battery_level < 100 :
			rospy.sleep(1)
			self._helper.battery_level += RECHARGE_PER_SECOND
			self._helper.battery_level = min(self._helper.battery_level, 100)
			rospy.loginfo('Battery level is: '+str(self._helper.battery_level))
		return 'recharged'


class ChooseNextRoom(State) :
	def __init__(self, helper):
		State.__init__(self, outcomes=['chosen'], output_keys=['next_room'])
		# Storing the helper for possible usage
		self._helper = helper
		
	def execute(self, userdata):
		# The robot time must be updated before choosing another room
		self._helper.update_robot_time()
		# Choosing the next room to visit
		if self._helper.battery_level <= 0 :
			# If the battery level is low, it must be charged
			room = RECHARGE_ROOM
			rospy.loginfo('Battery level is low, going into recharge room')
		else :
			# If the battery is not low, choose a room based on priority
			room, clss = self.choose_next_priotized_room()			
			rospy.loginfo('Choosing a room of type %s', clss)
		# Loggin chosen next room
		rospy.loginfo('The next room is: %s', room)
		userdata.next_room = room
		return 'chosen'
	
	def choose_next_priotized_room(self):
		# The robot moves in corridors when there are no near urgent rooms
		reachable_rooms = self._helper.retrieve_reachable_rooms()
		
		# Check if there is a reachable urgent room
		urgent_rooms = self._helper.retrieve_rooms_of_class('URGENT')
		rooms = list(set(reachable_rooms) & set(urgent_rooms))
		if rooms :
			return [rooms[random.randrange(len(rooms))], 'URGENT']
		
		# Check if there is a reachable corridor
		corridor_rooms = self._helper.retrieve_rooms_of_class('CORRIDOR')
		rooms = list(set(reachable_rooms) & set(corridor_rooms))
		if rooms :
			return  [rooms[random.randrange(len(rooms))], 'CORRIDOR']
		
		# Check if there is a reachable room
		corridor_rooms = self._helper.retrieve_rooms_of_class('ROOM')
		rooms = list(set(reachable_rooms) & set(corridor_rooms))
		return  [rooms[random.randrange(len(rooms))], 'ROOM']


class MoveToNextRoom(State) :
	def __init__(self, helper):
		State.__init__(self, outcomes=['moved'], input_keys=['next_room'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		# Taking the chosen room
		room = userdata.next_room
		# Moving to next room
		self._helper.move_robot_to_room(room)
		rospy.loginfo('Moved to room: %s', room)
		# Updating the time the robot visited the room
		self._helper.update_room_visited_time_before_exiting()
		return 'moved'
		

def main():
	# Initializing this ROS node
	rospy.init_node('TEST', log_level=rospy.INFO)
	
	# Creating a helper class to interract with Armor
	helper = ArmorHelper(ArmorClient('test_client', 'assignment_ontology'))
	
	# A state machine for deciding which room to choose and perform tasks
	sm = StateMachine(outcomes=[])
	with sm :
		# A state for the initialization of the robot
		StateMachine.add('INITIALIZATION', Initialization(helper),
				transitions={'initialized':'PERFORM_ROOM_TASKS'})
		
		# A state machine for performing a task in a room
		task_sm = StateMachine(outcomes=['task_completed'])
		with task_sm :
			# A task for deciding which is the task to perform in the room
			StateMachine.add('CHOOSE_ROOM_TASK', ChooseTaskForCurrentRoom(helper),
						transitions={'explore':'EXPLORE_TASK',
									 'recharge':'RECHARGE_TASK'})
			# This task makes the robot explore the current room
			StateMachine.add('EXPLORE_TASK', ExploreTask(helper),
						transitions={'explored':'task_completed'})
			# This task makes the robot recharge in the current room
			StateMachine.add('RECHARGE_TASK', RechargeTask(helper),
						transitions={'recharged':'task_completed'})
						
		# A state for performing tasks in the chosen room
		StateMachine.add('PERFORM_ROOM_TASKS', task_sm,
				transitions={'task_completed':'CHOOSE_NEXT_ROOM'})
		# A state for choosing the next room to reach
		StateMachine.add('CHOOSE_NEXT_ROOM', ChooseNextRoom(helper),
				transitions={'chosen':'MOVE_TO_NEXT_ROOM'},
				remapping={'next_room':'next_room'})
		# A state for reaching the next chosen room
		StateMachine.add('MOVE_TO_NEXT_ROOM', MoveToNextRoom(helper),
				transitions={'moved':'PERFORM_ROOM_TASKS'},
				remapping={'next_room':'next_room'})
						
	# In this case there is no output
	outcome = sm.execute()
	
	# Wait for ctrl-c to stop the application
	rospy.spin()
	
	

if __name__ == '__main__':
    try:
        main()
    except (Exception):
        sys.exit(0)
