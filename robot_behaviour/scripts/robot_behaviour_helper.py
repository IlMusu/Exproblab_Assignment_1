#!/usr/bin/env python

# The time libreary is necessary for getting the current time
import time
# Importing mutexes to handle multi-threading
from threading import Event

# Importing ARMOR libraries to easily interact with ARMOR
from armor_api.armor_client import ArmorClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_api.armor_manipulation_client import ArmorManipulationClient

# Importing ROS library for python
import rospy
import actionlib
from std_msgs.msg import UInt8, String
from robot_state_msgs.msg import MoveBetweenRoomsAction, MoveBetweenRoomsGoal
from robot_state_msgs.srv import ReferenceName

class RobotBehaviourHelper(object):
    '''
    Subscribes to:
        /battery_level (UInt8)
    Requests service to:
        /ontology_map/reference_name (ReferenceName)
    Requests action to :
        /robot_move (MoveBetweenRoomsAction)

    This is an helper class which provides some useful methods in order to abstract 
    the comunication with the ARMOR server in the particular context of the robot
    bahaviour. In order to provide this functionality, this class is heavily related 
    to the armor_api library.
    '''
    def __init__(self):
        '''
        This is the costructor method for the RobotBehaviourHelper class.
        1. Initilizes some internal variables.
        2. Creates a Subscriber to the /battery_level topic.
        3. Creates an ActionClietn for /robot_move ActionServer.
        '''
        # Subscriber for the battery level update
        self._battery_level = 100
        # A subscriber for the battery level
        self._battery_sub = rospy.Subscriber('/battery_level', UInt8, self._battery_level_callback)
        # An action client for moving the robot between rooms
        self._move_clt = actionlib.SimpleActionClient('/robot_move', MoveBetweenRoomsAction)

    
    def wait_for_ontology(self):
        '''
        This method makes the called wait until the ontology is fully loaded and
        then initializes the objects to communicate with ARMOR.
        This behaviour is obtained by calling the '/ontology_map/reference_name'
        service which responds only when the ontology is loaded.
        '''
        # Calling the service for obtaining the reference name
        rospy.wait_for_service('/ontology_map/reference_name')
        get_reference_name = rospy.ServiceProxy('/ontology_map/reference_name', ReferenceName)
        reference_name = get_reference_name().name
        # Creating objects for handling the comunication with ARMOR
        self._armor_client = ArmorClient('robot_behaviour', reference_name)
        self.onto_utils = ArmorUtilsClient(self._armor_client)
        self.onto_query = ArmorQueryClient(self._armor_client)
        self.onto_manip = ArmorManipulationClient(self._armor_client)
        
    
    def _battery_level_callback(self, msg):
        '''
        Args:
            msg (UInt8) : The message relative to /battery_level topic.
        
        This is callback for the /battery_level topic which provides
        the updated value of the battery level when it changes.
        '''
        # Storing the new battery level value
        self._battery_level = msg.data


    def get_battery_level(self):
        '''
        Returns:
            (int) : The current percentage of the battery
        '''
        return self._battery_level


    def retrieve_current_room(self):
        '''
        Returns:
            (string) : The name of the room in which 'Robot1' is currently in.

        Obtains from ARMOR the current room the 'Robot1' is in:
        1. Requests a reasoner synchronization to update the ontology.
        2. Requests the object property 'isIn' of 'Robot1'.
        3. Returns the room name obtained for property 'isIn'.
        '''
        # The reasoner must be started before querying something
        self.onto_utils.sync_buffered_reasoner()
        # Retrieving the current room the Robot1 is in
        room_name = self.onto_query.objectprop_b2_ind('isIn', 'Robot1')[0]
        return room_name
    
    
    def retrieve_reachable_rooms(self):
        '''
        Returns:
            (list) : The list of names of currently reachable rooms by 'Robot1'.

        Obtains from ARMOR the rooms reachable by 'Robot1':
        1. Requests a reasoner synchronization to update the ontology.
        2. Requests the object property 'canReach' of 'Robot1'.
        3. Returns the list of room names obtained for property 'canReach'.
        '''
        # The reasoner must be started before querying something
        self.onto_utils.sync_buffered_reasoner()
        # Retrieving reachable rooms
        rooms_names = self.onto_query.objectprop_b2_ind('canReach', 'Robot1')
        return rooms_names
    
    
    def retrieve_rooms_of_class(self, clss):
        '''
        Args:
            clss (string) : The class name of the rooms to retrieve.
        Returns:
            (list) : The list of the names of the rooms beloning to class clss.

        Obtains from ARMOR all the rooms belonging to a class:
        1. Requests a reasoner synchronization to update the ontology.
        2. Requests all the individuals belonging to a class.
        3. Returns the names of the obtained individuals.
        '''
        # The reasoner must be started before querying something
        self.onto_utils.sync_buffered_reasoner()
        # Retrieving all the rooms belongin to a class
        classes_names = self.onto_query.ind_b2_class(clss)
        return classes_names
    
    
    def move_robot_to_room(self, next_room):
        '''
        Args:
            next_room (string) : the next room the 'Robot1' needs to move into.
            
        Requests ARMOR to move the 'Robot1' in next_room:
        1. Retrieves the robot's current room.
        2. Sends a request to the /robot_move ActionServer to move the robot
           to the desired room. This is done until successdfull.
        3. Replaces the 'isIn' object property of 'Robot1' to next_room.
        '''
        # Retrieving the current room the Robot1 is in
        current_room = self.retrieve_current_room()
        reached_room = next_room
        # Requesting to the action server to move the robot
        #self._move_clt.wait_for_server()
        while False :
            goal = MoveBetweenRoomsGoal()
            goal.current_room = current_room
            goal.next_room = next_room
            self._move_clt.send_goal(goal)
            self._move_clt.wait_for_result()
            # Now we have the result
            result = self._move_clt.get_result()
            if not result.success :
                print('The robot failed to move to room '+next_room+'.')
            reached_room = result.current_room
        rospy.sleep(10)
        # Moving the robot in the ontology
        self.onto_manip.replace_objectprop_b2_ind('isIn', 'Robot1', reached_room, current_room)
    
    
    def retrieve_last_visited_time(self, room):
        '''
        Args:
            room (string) : The room from which to retrieve the time.
        Returns:
            (int) : The time in seconds at which 'Robot1' last visited the room.

        Obtains from ARMOR the time at which 'Robot1' visited the room:
        1. Requests a reasoner synchronization to update the ontology.
        2. Requests the data property 'visitedAt' of the room.
        3. Returns the time value obtained for property 'visitedAt'.
        '''
        # The reasoner must be started before querying something
        self.onto_utils.sync_buffered_reasoner()
        # Retrieving time
        data_id = self.onto_query.dataprop_b2_ind('visitedAt', room)[0]
        data = RobotBehaviourHelper._data_from_id(data_id)
        return data
    
    
    def retrieve_robot_time(self):
        '''
        Returns:
            (int) : The time in seconds of 'Robot1' representing the current time.

        Obtains from ARMOR the current time of 'Robot1' :
        1. Requests a reasoner synchronization to update the ontology.
        2. Requests the data property 'now' of the 'Robot1'.
        3. Returns the obtained value for property 'now'.
        '''
        # The reasoner must be started before querying something
        self.onto_utils.sync_buffered_reasoner()
        # Retrieving robot time
        data_id = self.onto_query.dataprop_b2_ind('now', 'Robot1')[0]
        data = RobotBehaviourHelper._data_from_id(data_id)
        return data
        
        
    def update_room_visited_time_before_exiting(self):
        '''
        Requests ARMOR to update the time at which 'Robot1' visited the
        room it is currently in with the value of the current time:
        1. Obtains the room the 'Robot1' is currently in.
        2. Obtains the old time the robot visited the room.
        2. Computes the current time in seconds.
        3. Replaces the 'visitedAt' data property of the room to the current time.
        '''
        # The robot is just exiting the room it is actually in
        room = self.retrieve_current_room()
        old = self.retrieve_last_visited_time(room)
        now = str(int(time.time()))
        # Update the last time the robot visited the room
        self.onto_manip.replace_dataprop_b2_ind('visitedAt', room, 'Long', now, old)
    
    
    def update_robot_time(self):
        '''
        Requests ARMOR to update the time value of 'Robot1':
        1. Obtains the old time of the 'Robot1'.
        2. Computes the current time in seconds.
        3. Replaces the 'now' data property of 'Robot1' to the current time.
        '''
        # Getting the robot outdated time and current time
        old = self.retrieve_robot_time()
        now = str(int(time.time()))
        # Update robot current time
        self.onto_manip.replace_dataprop_b2_ind('now', 'Robot1', 'Long', now, old)
    
    
    def update_robot_urgency_threshold(self, value):
        '''
        Requests ARMOR to update the urgencyThreshold value of 'Robot1':
        1. Obtains the old value of the urgencyThreshold.
        3. Replaces the old value with the new value.
        '''
        # The reasoner must be started before querying something
        self.onto_utils.sync_buffered_reasoner()
        # Getting the robot outdated time and current time
        old_id = self.onto_query.dataprop_b2_ind('urgencyThreshold', 'Robot1')[0]
        old = RobotBehaviourHelper._data_from_id(old_id)
        # Update robot current time
        self.onto_manip.replace_dataprop_b2_ind('urgencyThreshold', 'Robot1', 'Long', str(value), old)
    
    
    @staticmethod
    def _data_from_id(resource_id):
        '''
        Returns:
            (string) : The actual value of the resource.

        This is helper method to parse the value of a data property obtained 
        from ARMOR from a id to a more human-readable name.
        '''
        return resource_id.split('"')[1]


