#!/usr/bin/env python

# Importing AST library for safely evaluating strings
import ast
# Events for synchronizing threads
from threading import Event

# Importing ARMOR libraries to easily interact with ARMOR
from armor_api.armor_client import ArmorClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError

# Importing ROS library for python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from robot_state_msgs.srv import RoomPosition, RoomPositionResponse
from robot_state_msgs.srv import ReferenceName, ReferenceNameResponse

class OntologyMapBuilder(object):
    '''
    Provides services on:
        /ontology_map/reference_name (ReferenceName)
        /ontology_map/room_position (RoomPosition)
    
    Parameters:
        /ontology_reference : A string to use as the reference name of the ontology.
        /ontology_path : The global path of the default ontology.
        /ontology_uri : A string to use as the uri of the ontology.
        /map : The list of pairs (room, [doors]) for building the map.
        /room_positions : The list of pairs (room, position) for the rooms positions.
        /robot_room : The initial room at which the robot is located.
    
    This ROS node loads a default ontology into the ARMOR server and then creates
    the map specified by the user. The description of the map is provided through 
    parameters which are automatically decoded. Once the map if fully created,
    this node provides a service for providing the position of room given the name.
    '''
    def __init__(self) :
        '''
        This is the constructor method for the OntologyMapBuilder class.
        1. Creates a service for the "ontology_map/reference_name" service.
        2. Creates a service for the "ontology_map/room_position" service.
        3. Intializes object to communicate with ARMOR.
        4. Builds the map by calling the related method.
        '''
        # Initializing the ROS node
        rospy.init_node('ontology_map_builder', log_level=rospy.INFO)
        # Creating a Publisher for when the map is ready
        self._map_ready_pub = rospy.Service('ontology_map/reference_name', ReferenceName, self._onto_reference_service)
        # Creating a Service to provide the room positions
        self._rooms_pos_pub = rospy.Service('ontology_map/room_position', RoomPosition, self._room_position_service)
        # Initializing object to communicate with ARMOR
        self._onto_ref_name = rospy.get_param('/ontology_reference_name', 'ontology_reference')
        self._armor_client = ArmorClient('ontology_map_builder', self._onto_ref_name)
        self._onto_utils = ArmorUtilsClient(self._armor_client)
        self._onto_query = ArmorQueryClient(self._armor_client)
        self._onto_manip = ArmorManipulationClient(self._armor_client)
        # Starting to build the map
        self._building_complete = False
        self._building_complete_event = Event()
        self._build_ontology_map()

        
    def _build_ontology_map(self):
        '''
        This method creates the map specified by the user:
        1. Loads the default ontology into the ARMOR server.
        2. Decodes the description of the map and builds it.
        3. Disjoints the necessary individuals on the ontology
        4. Organizes the rooms positions by calling the related method.
        '''
        # Getting the map parameters from the Parameter Server
        onto_path = rospy.get_param('/ontology_path')
        onto_uri = rospy.get_param('/ontology_uri')
        encoded_map = ast.literal_eval(rospy.get_param('/map'))
        rooms_positions = ast.literal_eval(rospy.get_param('/room_positions'))
        robot_room = rospy.get_param('/robot_room', 'E')
        # Loading the default ontology into ARMOR
        rospy.loginfo("The base ontology is located at: "+onto_path)
        rospy.loginfo("Loading base ontology with uri: "+onto_uri)
        self._onto_utils.load_ref_from_file(onto_path, onto_uri, True)
        # Starting to create map
        rospy.loginfo("Starting to build map.")
        for [room, doors] in encoded_map :
            # Adding rooms with all the related doors
            for door in doors :
                self._onto_manip.add_objectprop_to_ind('hasDoor', room, door)
                rospy.loginfo("Added door "+door+" to room "+room+".")
            # Setting room properties to initial values
            self._onto_manip.add_dataprop_to_ind('visitedAt', room, 'Long', '0')
        # Disjoint between all the rooms
        self._onto_utils.sync_buffered_reasoner()
        self._onto_manip.disj_inds_of_class('ROOM')
        self._onto_manip.disj_inds_of_class('DOOR')
        # Positioning robot to specified room
        self._onto_manip.add_objectprop_to_ind('isIn', 'Robot1', robot_room)
        rospy.loginfo("Positioned robot in room "+robot_room+".")
        rospy.loginfo("Finished building map.")
        # Organizing room positions
        self._organize_rooms_positions(rooms_positions)
        rospy.loginfo("Finished organizing rooms positions.")
        rospy.sleep(20)
        # The building is now complete
        self._building_complete = True
        self._building_complete_event.set()


    def _organize_rooms_positions(self, room_positions):
        '''
        Arguments:
            room_positions (list) : The parsed list of pairs (rooms, position).
            
        This method parses the /room_positions argument into a dictionary. 
        This makes the retrieval of the positons more easy.
        '''
        self._room_positions = {}
        for [room, pos] in room_positions :
            self._room_positions[room] = Point(pos[0], pos[1], 0)
        
    
    def _onto_reference_service(self, request):
        '''
        This is the callback for the "/ontology_map/reference_name" service.
        It makes the client wait until the the map is complete, then responds
        with the reference name of the onotology.
        '''
        response = ReferenceNameResponse()
        if not self._building_complete :
            self._building_complete_event.wait()
        response.name = self._onto_ref_name
        return response


    def _room_position_service(self, request):
        '''
        This is the callback for the "/ontology_map/room_position" service.
        It checks that the requested room_name is valid and then fills the
        response with the position for the requested room.
        '''
        # Creating the response object
        response = RoomPositionResponse()
        # Filling the fields of the response
        response.valid = request.room_name in self._room_positions
        if response.valid :
            response.position = self._room_positions[request.room_name]
        # Returning the response to the client
        return response

        

if __name__ == '__main__' :
    # Creating and starting the map builder
    builder = OntologyMapBuilder()
    # Spinning to prevent quitting
    rospy.spin()


