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

ONTOLOGY_PATH = '/root/Desktop/topological_map.owl'
ONTOLOGY_URI = 'http://bnc/exp-rob-lab/2022-23'

class Initialization(State) :
	"""
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State used for initialization.
	|  Because of this functionality, it should be executed only once. 
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
			The outcome 'initialized' when the initialization is complete.
			
		|  This state initializes the robot behaviour by:
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
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State used for choosing the most suitable 
	|  task to be executed by the robot depending on the current room.
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['recharge', 'explore'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		"""
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'recharge' when the robot should recharge.
			The outcome 'explore' when the robot should explore the room.
			
		|  This states choosing the next task to be executed:
		|  1. If the battery is low, the robot must recharge.
		|  2. Otherwise the robot must explore the room.
		"""	
		# Check if robot needs recharging and this is a recharge room
		current_room = self._helper.retrieve_current_room()
		if self._helper.battery_level <= 0 and current_room == RECHARGE_ROOM:
			return 'recharge'
		
		# Otherwise another task is perfomed (exploration)
		return 'explore'

		
class ExploreTask(State):
	"""
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which simulates the robot
	|  exploring the room it is currenlty in.
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['explored'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		"""
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'explored' when exploration is complete.
			
		|  The robot explores the room for ROOM_EXPLORATION_TIME seconds.
		|  Then the battery of the robot is decreased to simulate the
		|  passing of time.
		"""	
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
	"""
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which simulates the robot
	|  recharging the designated recharge room.
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['recharged'])
		# Storing the helper for possible usage
		self._helper = helper
		
	def execute(self, userdata):
		"""
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'recharged' when the recharge is complete.
			
		|  The battery of the robot is recharged with a speed of RECHARGE_PER_SECOND. 
		|  The charge stops once the battery level is at the maximum value.
		"""	
		# Simulating the robot recharging
		rospy.loginfo('Starting recharge')
		while self._helper.battery_level < 100 :
			rospy.sleep(1)
			self._helper.battery_level += RECHARGE_PER_SECOND
			self._helper.battery_level = min(self._helper.battery_level, 100)
			rospy.loginfo('Battery level is: '+str(self._helper.battery_level))
		return 'recharged'


class ChooseNextRoom(State) :
	"""
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which decides the next room
	|  the robot should visit based on some policies:
	|  1. If the battery level of the robot is low, the robot should go to
	|     the room which has the recharging station.
	|  2. Otherwise, if the robot can reach a URGENT room, it should go there.
	|  3. Otherwise, if the robot can reach a CORRIDOR, it should go there.
	|  4. Otherwise, the robot should go in any reachable room.
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['chosen'], output_keys=['next_room'])
		# Storing the helper for possible usage
		self._helper = helper
		
	def execute(self, userdata):
		"""
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'chosen' when a room has been selected.
			
		|  If the battery of robot is not enough to reach another room,
		|  makes the robot go into the room with the recharging station.
		|  Otherwise, makes the robot go into another room depending on
		|  the priority of the room.
		"""	
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
		"""
		Returns:
			The room which the robot can reach and has the higher priority.
		
		The priority of the rooms is decided by the following policy: 
		|  1. If the robot can reach a URGENT room, it should go there.
		|  2. Otherwise, if the robot can reach a CORRIDOR, it should go there.
		|  3. Otherwise, the robot should go in any reachable room.
		"""
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
		
		# Go to any reachable room
		return  [reachable_rooms[random.randrange(len(reachable_rooms))], 'ROOM']


class MoveToNextRoom(State) :
	"""
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which makes the robot move
	|  to the selected room.
    """
	def __init__(self, helper):
		State.__init__(self, outcomes=['moved'], input_keys=['next_room'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		"""
		Returns:
			The outcome 'moved' when the robot moved to the selected room.
		
		| Makes the robot move from the current room to the selected room.
		"""
		# Taking the chosen room
		room = userdata.next_room
		# Moving to next room
		self._helper.move_robot_to_room(room)
		rospy.loginfo('Moved to room: %s', room)
		# Updating the time the robot visited the room
		self._helper.update_room_visited_time_before_exiting()
		return 'moved'
		

def main():
	"""
	| In this method the state machine which defines the robot behaviour is created.
	| The state machine provides an INITIALIZATION state for initializing all the
	| necessary parameters. Then, a inner state machine is created for handling the
	| tasks that the robot may perfom in the room. In this case there are only two 
	| possible states, which are the EXPLORE_TASK state and the RECHARGE_TASK state.
	| After perfomrming the task, another room is selected via the CHOOSE_NEXT_ROOM 
	| state and then the robot is move there via the MOVE_TO_NEXT_ROOM state.
	"""
	# Initializing this ROS node
	rospy.init_node('robot_behaviour', log_level=rospy.INFO)
	
	# Creating a helper class to interract with Armor
	helper = ArmorHelper(ArmorClient('rb_client', 'assignment_ontology'))
	
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
