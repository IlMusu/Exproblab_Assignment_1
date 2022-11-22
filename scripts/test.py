#!/usr/bin/env python

# Importing sys to handling correct exiting of the program
import sys
# The random library is necessary for debugging
import random
# The time libreary is necessary for getting the current time
import time

# Imporing the ROS library for python
import rospy
from rospy.exceptions import ROSInterruptException

# Importing SMACH to create FINITE STATE MACHINES
from smach import StateMachine, State

# Importing ARMOR libraries to easily interact with ARMOR
from armor_api.armor_client import ArmorClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_exceptions import ArmorServiceCallError

ROOM_EXPLORATION_TIME = 10
RECHARGE_ROOM = 'E'
RECHARGE_PER_SECOND = 10

class ArmorHelper(object):
	def __init__(self, armor_client):
		self._armor_client = armor_client
		self.armor_utils_client = ArmorUtilsClient(armor_client)
		self.armor_query_client = ArmorQueryClient(armor_client)
		self.armor_manipulation_client = ArmorManipulationClient(armor_client)
	
	def load_ontology(self):
		# Loading ontology and syncing reasoner
		self.armor_utils_client.load_ref_from_file(ONTOLOGY_PATH, ONTOLOGY_URI, True)
	
	def move_robot_to_room(self, next_room):
		# Retrieving the current room the Robot1 is in
		current_room = self.retrieve_current_room()
		# Actually moving the robot to another room
		self.armor_manipulation_client.replace_objectprop_b2_ind('isIn', 'Robot1', next_room, current_room)
	
	def retrieve_current_room(self):
		# The reasoner must be started before querying something
		self.armor_utils_client.sync_buffered_reasoner()
		# Retrieving the current room the Robot1 is in
		room_id = self.armor_query_client.objectprop_b2_ind('isIn', 'Robot1')[0]
		room = ArmorHelper._name_from_id(room_id)
		return room
	
	def retrieve_reachable_rooms(self):
		# The reasoner must be started before querying something
		self.armor_utils_client.sync_buffered_reasoner()
		# Retrieving reachable rooms
		rooms_ids = self.armor_query_client.objectprop_b2_ind('canReach', 'Robot1')
		rooms = list(map(ArmorHelper._name_from_id, rooms_ids))
		return rooms
	
	def retrieve_rooms_of_class(self, clss):
		# The reasoner must be started before querying something
		self.armor_utils_client.sync_buffered_reasoner()
		# Retrieving all the rooms belongin to a class
		classes_ids = self.armor_query_client.ind_b2_class(clss)
		classes = list(map(ArmorHelper._name_from_id, classes_ids))
		return classes
	
	def retrieve_last_visited_time(self, room):
		# The reasoner must be started before querying something
		self.armor_utils_client.sync_buffered_reasoner()
		# Retrieving time
		data_id = self.armor_query_client.dataprop_b2_ind('visitedAt', room)[0]
		data = ArmorHelper._data_from_id(data_id)
		return data
	
	def retrieve_robot_time(self):
		# The reasoner must be started before querying something
		self.armor_utils_client.sync_buffered_reasoner()
		# Retrieving robot time
		data_id = self.armor_query_client.dataprop_b2_ind('now', 'Robot1')[0]
		data = ArmorHelper._data_from_id(data_id)
		return data
		
	def update_room_visited_time_before_exiting(self):
		# The robot is just exiting the room it is actually in
		room = self.retrieve_current_room()
		old = self.retrieve_last_visited_time(room)
		now = str(int(time.time()))
		# Update the last time the robot visited the room
		self.armor_manipulation_client.replace_dataprop_b2_ind('visitedAt', room, 'Long', now, old)
	
	def update_robot_time(self):
		# Getting the robot outdated time and current time
		old = self.retrieve_robot_time()
		now = str(int(time.time()))
		# Update robot current time
		self.armor_manipulation_client.replace_dataprop_b2_ind('now', 'Robot1', 'Long', now, old)
	
	@staticmethod
	def _name_from_id(resource_id):
		return resource_id.split('#')[1][0:-1]
	
	@staticmethod
	def _data_from_id(resource_id):
		return resource_id.split('"')[1]
		

class Initialization(State) :
	def __init__(self, helper):
		State.__init__(self, outcomes=['initialized'])
		# Storing th helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		# Loading ontology into armor 
		self._helper.load_ontology()
		# Initializing battery level at the maximum
		self._helper.battery_level = 100
		return 'initialized'


class ChooseRoomTask(State) :
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
			StateMachine.add('CHOOSE_ROOM_TASK', ChooseRoomTask(helper),
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
