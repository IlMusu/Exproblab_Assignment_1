# The time libreary is necessary for getting the current time
import time
# Importing mutexes to handle multi-threading
from threading import Lock

# Importing ARMOR libraries to easily interact with ARMOR
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_api.armor_manipulation_client import ArmorManipulationClient

# Importing ROS library for python
import rospy
from std_msgs.msg import UInt8

class RobotBehaviourHelper(object):
    '''
    |  This is an helper class which provides some useful methods in order to
    |  abstract the comunication with the ARMOR server. In order to provide this
    |  behaviour, this class is heavily related to the armor_api library.
    '''
    def __init__(self, armor_client):
        '''
        Args:
            armor_client : An instance of ArmorClient in order to communicate with ARMOR.
        
        |  This is the costructor method in which some variables are initialized.
        |  Also, a subscriber to the /battery_level topic is created. 
        '''
        # Creating objects for handling the comunication with ARMOR
        self._armor_client = armor_client
        self.armor_utils_client = ArmorUtilsClient(armor_client)
        self.armor_query_client = ArmorQueryClient(armor_client)
        self.armor_manipulation_client = ArmorManipulationClient(armor_client)
        # Subscriber for the battery level update
        self._battery_level_mutex = Lock()
        self._battery_level = 100
        self.battery_low = 10
        self.battery_enough = 80
        rospy.Subscriber('/battery_level', UInt8, self._battery_level_callback)

    def load_ontology(self, path, uri):
        '''
        | Loads the specified ontology into the ARMOR server.
        '''
        self.armor_utils_client.load_ref_from_file(path, uri, True)
    
    def _battery_level_callback(self, msg):
        '''
        Args:
            msg: The message relative to /battery_level topic.
        
        |  This is callback for the /battery_level topic which provides
        |  the updated value of the battery level when it changes.
        |  The value is stored in a variable which is protected by mutexes
        |  to prevent concurrency problems. 
        '''
        # Storing the new battery level value
        # Using locks to protect from concurrency problems
        self._battery_level_mutex.acquire()
        self._battery_level = msg.data
        self._battery_level_mutex.release()

    def is_battery_under(self, threshold):
        '''
        Args:
            threshold: The threshold for battery low.
        Returns:
            If the battery level should be considered low.

        |  This is a helper method which compares the last value received
        |  for the battery level with the threshold value for when it should
        |  considered.
        '''
        self._battery_level_mutex.acquire()
        is_battery_under = self._battery_level <= threshold
        self._battery_level_mutex.release()
        return is_battery_under

    def retrieve_current_room(self):
        '''
        Returns:
            The name of the room in which 'Robot1' is currently in.

        |  Obtains from ARMOR the current room the 'Robot1' is in:
        |  1. Requests a reasoner synchronization to update the ontology.
        |  2. Requests the object property 'isIn' of 'Robot1'.
        |  3. Returns the room name obtained for property 'isIn'.
        '''
        # The reasoner must be started before querying something
        self.armor_utils_client.sync_buffered_reasoner()
        # Retrieving the current room the Robot1 is in
        room_name = self.armor_query_client.objectprop_b2_ind('isIn', 'Robot1')[0]
        return room_name
    
    def retrieve_reachable_rooms(self):
        '''
        Returns:
            The list of names of currently reachable rooms by 'Robot1'.

        |  Obtains from ARMOR the rooms reachable by 'Robot1':
        |  1. Requests a reasoner synchronization to update the ontology.
        |  2. Requests the object property 'canReach' of 'Robot1'.
        |  3. Returns the list of room names obtained for property 'canReach'.
        '''
        # The reasoner must be started before querying something
        self.armor_utils_client.sync_buffered_reasoner()
        # Retrieving reachable rooms
        rooms_names = self.armor_query_client.objectprop_b2_ind('canReach', 'Robot1')
        return rooms_names
    
    def retrieve_rooms_of_class(self, clss):
        '''
        Args:
            clss : The class name of the rooms to retrieve.
        Returns:
            The list of the names of the rooms beloning to class clss.

        |  Obtains from ARMOR all the rooms belonging to a class:
        |  1. Requests a reasoner synchronization to update the ontology.
        |  2. Requests all the individuals belonging to a class.
        |  3. Returns the names of the obtained individuals.
        '''
        # The reasoner must be started before querying something
        self.armor_utils_client.sync_buffered_reasoner()
        # Retrieving all the rooms belongin to a class
        classes_names = self.armor_query_client.ind_b2_class(clss)
        return classes_names
    
    def move_robot_to_room(self, next_room):
        '''
        Args:
            next_room : the next room the 'Robot1' needs to move into.
            
        |  Requests ARMOR to move the 'Robot1' in next_room:
        |  1. Retrieves the robot's current room.
        |  2. Replaces the 'isIn' object property of 'Robot1' to next_room.
        '''
        # Retrieving the current room the Robot1 is in
        current_room = self.retrieve_current_room()
        # Actually moving the robot to another room
        self.armor_manipulation_client.replace_objectprop_b2_ind('isIn', 'Robot1', next_room, current_room)
    
    def retrieve_last_visited_time(self, room):
        '''
        Args:
            room : The room from which to retrieve the time.
        Returns:
            The time in seconds at which 'Robot1' last visited the room.

        |  Obtains from ARMOR the time at which 'Robot1' visited the room:
        |  1. Requests a reasoner synchronization to update the ontology.
        |  2. Requests the data property 'visitedAt' of the room.
        |  3. Returns the time value obtained for property 'visitedAt'.
        '''
        # The reasoner must be started before querying something
        self.armor_utils_client.sync_buffered_reasoner()
        # Retrieving time
        data_id = self.armor_query_client.dataprop_b2_ind('visitedAt', room)[0]
        data = RobotBehaviourHelper._data_from_id(data_id)
        return data
    
    def retrieve_robot_time(self):
        '''
        Returns:
            The time in seconds of 'Robot1' representing the current time.

        |  Obtains from ARMOR the current time of 'Robot1' :
        |  1. Requests a reasoner synchronization to update the ontology.
        |  2. Requests the data property 'now' of the 'Robot1'.
        |  3. Returns the obtained value for property 'now'.
        '''
        # The reasoner must be started before querying something
        self.armor_utils_client.sync_buffered_reasoner()
        # Retrieving robot time
        data_id = self.armor_query_client.dataprop_b2_ind('now', 'Robot1')[0]
        data = RobotBehaviourHelper._data_from_id(data_id)
        return data
        
    def update_room_visited_time_before_exiting(self):
        '''
        |  Requests ARMOR to update the time at which 'Robot1' visited the
        |  room it is currently in with the value of the current time:
        |  1. Obtains the room the 'Robot1' is currently in.
        |  2. Obtains the old time the robot visited the room.
        |  2. Computes the current time in seconds.
        |  3. Replaces the 'visitedAt' data property of the room to the current time.
        '''
        # The robot is just exiting the room it is actually in
        room = self.retrieve_current_room()
        old = self.retrieve_last_visited_time(room)
        now = str(int(time.time()))
        # Update the last time the robot visited the room
        self.armor_manipulation_client.replace_dataprop_b2_ind('visitedAt', room, 'Long', now, old)
    
    def update_robot_time(self):
        '''
        |  Requests ARMOR to update the time value of 'Robot1' which
        |  represents the current time:
        |  1. Obtains the old time of the 'Robot1'.
        |  2. Computes the current time in seconds.
        |  3. Replaces the 'now' data property of 'Robot1' to the current time.
        '''
        # Getting the robot outdated time and current time
        old = self.retrieve_robot_time()
        now = str(int(time.time()))
        # Update robot current time
        self.armor_manipulation_client.replace_dataprop_b2_ind('now', 'Robot1', 'Long', now, old)
    
    @staticmethod
    def _data_from_id(resource_id):
        '''
        |  This is helper method to parse the name of a data property obtained 
        |  from ARMOR to a more human-readable name.
        '''
        return resource_id.split(''')[1]

