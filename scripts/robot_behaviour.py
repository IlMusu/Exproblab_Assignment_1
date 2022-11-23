#!/usr/bin/env python

# The RANDOM library is necessary for debugging
import random
# Importing ROS library for python
import rospy
# Importing SMACH to create FINITE STATE MACHINES
from smach import StateMachine, State
# Importing HELPER to abstract interaction with ARMOR
from armor_api.armor_client import ArmorClient
from robot_behaviour_helper import RobotBehaviourHelper


class Initialization(State) :
	'''
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State used for the initialization
    |  of the parameters which define the behaviour of the robot.
	|  Because of this functionality, it should be executed only once. 
    '''
	def __init__(self, helper):
		State.__init__(self, outcomes=['initialized'])
		# Storing th helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		'''
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'initialized' when the initialization is complete.
			
		|  This state initializes the robot behaviour by:
		|  1. Loading the ontology into the ARMOR server.
		|  2. Setting the battery level of the robot the 100%.
		|  3. Loading the parameters set in the launch file.
		'''
		# Loading ontology into armor 
		onto_path = rospy.get_param('/ontology_path')
		onto_uri = rospy.get_param('/ontology_uri')
		self._helper.load_ontology(onto_path, onto_uri)
		# Setting some parameters to personalize behaviour
		self._helper.battery_level = 100
		self._helper.recharge_rooms = _strlist_to_list(rospy.get_param('/recharge_rooms'))
		self._helper.recharge_per_second = rospy.get_param('/recharge_per_second')
		self._helper.rooms_priority = _strlist_to_list(rospy.get_param('/rooms_priority'))
		self._helper.exploration_seconds = rospy.get_param('/exploration_seconds')
		
		return 'initialized'


class ChooseTaskForCurrentRoom(State) :
	'''
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State used for choosing the most 
    |  suitable task to be executed by the robot. In particular, the
    |  task is chosen consider some parameters of the robot and the
    |  capabilities of the room.
    '''
	def __init__(self, helper):
		State.__init__(self, outcomes=['recharge', 'explore'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		'''
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'recharge' when the robot should recharge.
			The outcome 'explore' when the robot should explore the room.
			
		|  This states choosing the next task to be executed:
		|  1. If the battery is low and there is a reacharge station in
		|     the room, then the robot must recharge.
		|  2. Otherwise the robot must explore the room.
		'''	
		# Check if robot needs recharging and this is a recharge room
		current_room = self._helper.retrieve_current_room()
		is_room_for_charge = current_room in self._helper.recharge_rooms
		if self._helper.battery_level <= 0 and is_room_for_charge:
			return 'recharge'
		
		# Otherwise another task is perfomed (exploration)
		return 'explore'

		
class ExploreTask(State):
	'''
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which simulates the robot
	|  exploring the room it is currenlty in.
    '''
	def __init__(self, helper):
		State.__init__(self, outcomes=['explored'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		'''
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'explored' when exploration is complete.
			
		|  The robot explores the room for the amount of seconds that
		|  the user specified in the launch file. Then, the battery of 
		|  the robot is drained to simulate the passing of time.
		'''	
		# Simulating the robot exploring the room
		rospy.loginfo('Exploring room (%i seconds)', self._helper.exploration_seconds)
		#rospy.sleep(self._helper.exploration_seconds)
		input() # TODO remove this
		rospy.loginfo('Finished exploring room')
		# The robot moved in the room, it looses some battery
		self._helper.battery_level -= random.randrange(5, 15)
		self._helper.battery_level = max(self._helper.battery_level, 0)
		rospy.loginfo('Battery level is: '+str(self._helper.battery_level))
		return 'explored'


class RechargeTask(State):
	'''
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which simulates the robot
	|  recharging into the designated recharge room.
    '''
	def __init__(self, helper):
		State.__init__(self, outcomes=['recharged'])
		# Storing the helper for possible usage
		self._helper = helper
		
	def execute(self, userdata):
		'''
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'recharged' when the recharge is complete.
			
		|  The battery of the robot is recharged with a speed defined by the user.
		|  The charge stops once the battery level is at the maximum value.
		'''	
		# Simulating the robot recharging
		rospy.loginfo('Starting recharge')
		while self._helper.battery_level < 100 :
			rospy.sleep(1)
			self._helper.battery_level += self._helper.recharge_per_second
			self._helper.battery_level = min(self._helper.battery_level, 100)
			rospy.loginfo('Battery level is: '+str(self._helper.battery_level))
		return 'recharged'


class ChooseNextRoom(State) :
	'''
    Args:
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which decides the next room
	|  the robot should visit based on the following policy: if the
	|  robot has not enough battery to reach another room, a room which
	|  contains a recharge station is chosen. Otherwise, the reachable
	|  room with the highest priority is chosen.
    '''
	def __init__(self, helper):
		State.__init__(self, outcomes=['chosen'], output_keys=['next_room'])
		# Storing the helper for possible usage
		self._helper = helper
		
	def execute(self, userdata):
		'''
		Args:
			userdata : The data of this State.
		Returns:
			The outcome 'chosen' when a room has been selected.
			
		|  If the battery of robot is not enough to reach another room,
		|  makes the robot go into the room with the recharging station.
		|  Otherwise, makes the robot go into another room depending on
		|  the rooms priorities.
		'''	
		# The robot time must be updated before choosing another room
		self._helper.update_robot_time()
		# Choosing the next room to visit
		if self._helper.battery_level <= 0 :
			# If the battery level is low, it must be charged
			room = _rand_in_list(self._helper.recharge_rooms)
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
		'''
		Returns:
			1. The reachable room with the highest priority.
			2. The class name of the room.
		
		|  The rooms priorities are decided by user with the dedicated parameter.
		|  This method loops over all the priorities in order and checks if
		|  there is a reachable room of the given priority.
		|  If no room is found, returns any reachable room.
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
        helper : An instance of ArmorHelper to communicate with ARMOR.
        
    |  This class is an instance of State which makes the robot move
	|  to the selected room.
    '''
	def __init__(self, helper):
		State.__init__(self, outcomes=['moved'], input_keys=['next_room'])
		# Storing the helper for possible usage
		self._helper = helper
	
	def execute(self, userdata):
		'''
		Returns:
			The outcome 'moved' when the robot moved to the selected room.
		
		| Makes the robot move from the current room to the selected room.
		'''
		# Taking the chosen room
		room = userdata.next_room
		# Moving to next room
		self._helper.move_robot_to_room(room)
		rospy.loginfo('Moved to room: %s', room)
		# Updating the time the robot visited the room
		self._helper.update_room_visited_time_before_exiting()
		return 'moved'




class RobotBehaviour(object):
	def __init__(self):
		# Initializing a ROS node which represents the robot behaviour
		rospy.init_node('robot_behaviour', log_level=rospy.INFO)
		# Creating a instance of the helper
		client_name = rospy.get_param('/client_name')
		reference_name = rospy.get_param('/reference_name')
		self._helper = RobotBehaviourHelper(ArmorClient(client_name, reference_name))

	def generate_behaviour(self):
		# A state machine that describes the behaviour of the robot
		behaviour_sm = StateMachine(outcomes=[])
		with behaviour_sm :
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
		
		return behaviour_sm


def _strlist_to_list(strlist):
	formatted = strlist.strip('][').replace('"', '').replace("'", '').split(',')
	return list(map(str.strip, formatted))


def _rand_in_list(lst):
	return lst[random.randrange(len(lst))]


def main():
	# Creating an instance of the RobotBehaviour
	robot_behaviour = RobotBehaviour()
	# Actually generating the behaviour of the robot
	behaviour_sm = robot_behaviour.generate_behaviour()
	# Executing the behaviour of the robot
	outcome = behaviour_sm.execute()
	
	# Wait for ctrl-c to stop the application
	rospy.spin()


if __name__ == '__main__':
	main()



