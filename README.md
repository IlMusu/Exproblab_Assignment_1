# EXPROBLAB - ASSIGNMENT 1
Author : Mattia Musumeci 4670261@studenti.unige.it  
  
This is the first assignment developed for the <b>Experimental Robotics Laboratory</b> course of the University of Genoa.  
At this <b>[link](http://documentaiton)</b> it is possible to find the documentation for the software contained in this repository.  

## 1. INTRODUCTION
The scenario involves a <b>robot</b> deployed in an indoor environment for <b>surveillance purposes</b> whose objective is to visit the different environment locations and explore them for a given amount of time. In this context, the robot in equipped with a rechargeable battery which needs to be recharged in a specific location when required.  
  
The software contained in this repository has been developed for <b>[ROS Noetic 1.15.9](http://wiki.ros.org/noetic)</b>.  
The <b>Robot Operating System (ROS)</b> is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.  
  
The behavior of the robot has been described with a finite state machine developed for <b>[SMACH](http://wiki.ros.org/smach)</b>.  
The <b>SMACH</b> library is a task level architecture for describing and executing complex behaviors. At its core, it is a ROS-independent python library to build hierarchical state machines and the interaction with the ROS environment is obtained by implementing dedicated states inside the finite state machine.  
  
The ontology concepts and reasoning have been implemented with <b>[ARMOR](https://github.com/EmaroLab/armor)</b>.  
A <b>ROS Multi-Ontology Reference (ARMOR)</b> is a powerful and versatile management system for single and multi-ontology architectures under ROS. It allows to load, query and modify multiple ontologies and requires very little knowledge of OWL APIs and Java.  

### Ontology
The ontology developed for this assignment is composed by the following class hierarchy :

    Thing
	  ├──── DOOR
	  ├──── LOCATION
	  │            ├──── CORRIDOR 
	  │            ├──── ROOM
	  │            └──── URGENT
	  └──── ROBOT

In which:
- a DOOR is a thing.
- a LOCATION is thing that has the `visitedAt` property.
  - a CORRIDOR is a LOCATION that has at least 2 `hasDoor` properties.
  - a ROOM is a LOCATION that has at least 2 `hasDoor` properties.
  - a URGENT is a LOCATION.
- a ROBOT is a thing.
  
In the following are explained all the properties used in the ontology :
- The `visitedAt` data property contains the last time in seconds at which the robot visited the thing.
- The `hasDoor` object property contains the name of a DOOR.
- The `now` data property contains the value of the current time in seconds.
- The `isIn` object property contains the name of a LOCATION.
- The `urgencyThreshold` data property of contains a delta time value in seconds.
  
Notice that the last three properties are relative to the ROBOT.
  
The following pseudo-code explains when a LOCATION becomes URGENT to be visited by the ROBOT:
```bash
ROBOT.now - LOCATION.visitedAt > ROBOT.urgencyThreshold
```

### Environment
The indoor environment considered in this assignment is the following one:
- The ROOMs are `R1`, `R2`, `R3`, `R4`.
- The CORRIDORs are `E`, `C1`, `C2`.
- The LOCATION `E` contains the recharging station.
- The LOCATION `E` is the one in which the robot is positioned.

<p align="center">
	<img src="https://i.imgur.com/SQZ4ySu.png" />
</p>

### Surveillance Policy
In order to explore the environment, the robot must follow a <b>surveillance policy</b> which is fully described by the following rules ordered by decreasing priority:
1. The robot should go to the E location when the battery is low.
2. The robot should go into any reachable URGENT locations.
3. The robot prefers to stay in CORRIDORS.

## 2. INSTALLATION AND RUNNING
### Installation
The software contained in this repository is composed of two ROS packages.  
Therefore, in order to install the software, it is necessary to create a workspace:  
```bash
mkdir -p [workspace_name]/src
cd [workspace_name]/
catkin_make
```
Then, clone this repository inside the src folder just created:
```bash
cd src/
git clone [this_repo_link] .
```

Then, rebuild the workspace by returning to the workspace folder:
```bash
cd ..
catkin_make
```

The setup.bash file must be sourced so that ROS can find the workspace.  
To do this, the following line must be added at the end of the .bashrc file:
```bash
source [workspace_folder]/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:[workspace_folder]/src
```
### Running
In order to run the scripts, it is necessary to first run the ROS master.  
Open a new console and run the following command:
```bash
roscore
```

Then, it is necessary to run the ARMOR library.  
Open a new console and run the following command:
```bash
rosrun armor execute it.emarolab.armor.ARMORMainService
```

Then, open a new console and run the robot_state node:  
Notice that only one between the three shown commands are necessary to run the node. The last two commands show how it is possible to use command line arguments to modify the execution mode of the controllers.
To see the complete list of arguments for this launch file, look at [this]() section.
```bash
roslaunch robot_behaviour robot_state.launch
# roslaunch robot_behaviour robot_state.launch move_controller_execution_mode:="RANDOM"
# roslaunch robot_behaviour robot_state.launch battery_controller_execution_mode:="MANUAL"
```

Finally, open a new console and run the robot_state node:  
To see the complete list of arguments for this launch file, look at [this]() section.
```bash
roslaunch robot_behaviour robot_behaviour.launch
```

## 3. SOFTWARE ARCHITECTURE
### Component Diagram
In the <b>component diagram</b> are shown all the <b>blocks</b> and <b>interfaces</b> that have been used or developed in order to obtain the desired software architecture.

<p align="center">
	<img src="https://i.imgur.com/qKbQabN.png" />
</p>

- The `ontology_map_builder` node loads the default ontology into ARMOR and builds the map following the user requests. It also contains a mapping between each room and its position with respect to the world frame. Notice that in this context, the "position of a room" is defined as a point inside the room that the robot is able to reach.  It interacts with:
   - The `armor_service` library through the <b>/armor_interface_srv</b> service.  
- The `robot_behavior` node contains the state machine that describes the desired behavior of the robot. The functionality of this node is based on the interaction with other nodes. In particular, it interacts with:  
  - The `ontology_map_builder` node through the <b>/ontology_map/reference_name</b> service.  
  - The `ontology_map_builder` node through the <b>/ontology_map/room_position</b> service.  
  - The `armor_service` library through the <b>/armor_interface_srv</b> service.  
  - The `battery_controller` node through the <b>/battery_level</b> message.  
  - The `planner_controller` node through the <b>/compute_path</b> action.  
  - The `motion_controller` node through the <b>/follow_path</b> action.  
- The `battery_controller` node controls the level of the battery. It interacts with:  
  - The `robot_behaviour` node through the <b>/battery_level</b> message.  
- The `planner_controller` node constructs a path between two points. It interacts with:  
  - The `robot_behaviour` node through the <b>/compute_path</b> message.  
- The `motion_controller` node controls the movement of the robot. It interacts with:  
  - The `robot_behaviour` node through the <b>/follow_path</b> message.  

A more detailed explanation of the use of the interfaces is available <b>[here](#ros-messages,-services-and-actions)</b>.  
A more detailed explanation of the controllers is available <b>[here](#ros-messages,-services-and-actions)</b>. 

### States Diagram
This <b>state diagram</b> shows the state machine representing the desired behavior of the robot. In particular, all the possible states and transitions are shown.

<p align="center">
	<img src="https://i.imgur.com/nynYen4.png" />
</p>

- Inside the `INITIALIZATION` state, the following operations are performed:  
  1) Waits until the ontology is fully loaded onto ARMOR.  
  2) Initializes some parameters for the robot.  
  3) Returns the outcome <b>"initialized"</b>.  
- The `PERFORM_ROOM_TASK` state is a sub state machine containing the following states:  
  - Inside the `CHOOSE_ROOM_TASK` state, the following operations are performed:  
    1) If the battery needs to be recharged, returns the outcome <b>"recharge"</b>.  
    2) Otherwise, returns the outcome <b>"explore"</b>  
  - Inside the `EXPLORE_TASK` state, the following operations are performed:  
    1) Waits for a predefined number of seconds to simulate the robot exploring the room.  
    2) Returns the outcome <b>"explored"</b>.  
 - Inside the `RECHARGE_TASK` state, the following operations are performed:  
    1) Waits until the battery level is considered enough to stop recharging.  
    2) Returns the outcome <b>"recharged"</b>.  
- Inside the `CHOOSE_NEXT_ROOM` state, the following operations are performed:  
  1) If the battery needs to be recharged, selects the recharging room as the next room.  
  2) Otherwise, selects the next room on the basis of the <b>[surveillance policy](#surveillance-policy)</b>.  
  3) Returns the outcome <b>"chosen"</b>.  
- Inside the `MOVE_TO_NEXT_ROOM` state, the following operations are performed:  
  1) Updates the time at which the robot visited the room it is leaving.  
  2) Moves the robot from the current room to the selected room.  
  3) Returns the outcome <b>"moved"</b>.  

### States Diagram
This <b>sequence diagram</b> shows a possible execution of the software contained in this repository. More in details, this diagram shows the execution in time of all the nodes and the requests/responses between them.

<p align="center">
	<img src="https://i.imgur.com/bgzpput.png" />
</p>

One thing to immediately notice in this diagram is that every time something is retrieved from the armor_server node, the reasoner is updated so that the retrieved value is always updated. This should be shown in the diagram but, for simplicity of visualization, is omitted.
The middle horizontal line shows that at the time of the "CHOOSE_ROOM_TASK" state, the robot is surely not in the E room where it has the capability of recharging (that is because it moved only once and initially it was in the E room). Therefore, the diagram would be identical to before. Instead, it is shown what would happen the next time the robot moves, hence, when the robot returns to the E room and the state machine starts the "RECHARGE_TASK" state.  

### ROS Messages, Services And Actions
In order to develop the interfaces between the components:  
- The <b>ontology_map_builder</b> node which:  
  - Provides the <b>`/ontology_map/reference_name`</b> service, of type `ReferenceName.srv`, to provide the reference name of the ontology that is loaded into ARMOR. This is done only once the ontology is fully created and loaded.  
  - Provides the <b>`/ontology_map/room_position`</b> service, of type `RoomPosition.srv`, to provide a position inside the requested room. The position is measured with respect to the world frame.  
- The <b>robot_behaviour</b> node which:  
  - Uses the <b>`/ontology_map/reference_name`</b> service, of type `ReferenceName.srv`, to obtain the reference name of the ontology that is loaded into ARMOR.  
  - Uses the <b>`/ontology_map/room_position`</b> service, of type `RoomPosition.srv`, to obtain a position inside the specified room. The position is measured with respect to the world frame.
  - Uses the <b>`/armor_interface_srv`</b> service, of type `ArmorDirective.srv`, to interact with the ARMOR in order to modify the ontology and retrieve knowledge.  
  - Subscribes to the <b>`/battery_level`</b> topic, of type `UInt8.msg`, to retrieve the updated battery level.  
  - Creates a client for the <b>`/compute_path`</b> action server, of type  ` ComputePath.action`, in order to compute the path between the current position of the robot and a goal position.  
  - Creates a client for the <b>`/follow_path`</b> action server, of type  ` FollowPath.action`, in order to make the robot follow the previously computed path until the last position of the path is reached.  
- The <b>battery_controller</b> node which:  
   - Publishes to the <b>`/battery_level`</b> topic, of type `UInt8.msg`, the updated value of the battery level.  
- The <b>planner_controller</b> node which:  
  - Creates a <b>`/compute_path`</b> action server, of type `ComputePath.action` in order to obtain the start and goal positions and computing the related path.  
- The <b>motion_controller</b> node which:  
  - Creates a <b>`/follow_path`</b> action server, of type ` FollowPath.action`, in order to obtain the path that the robot has to follow and make the robot follow it until the final position is reached.  

A more detailed description of the custom messages, actions and services that have been created for this architecture can be found in the related files [ReferenceName](https://github.com/IlMusu/Exproblab_Assignment_1/blob/master/robot_state_msgs/srv/ReferenceName.srv),  [RoomPosition](https://github.com/IlMusu/Exproblab_Assignment_1/blob/master/robot_state_msgs/srv/RoomPosition.srv),  [ComputePath](https://github.com/IlMusu/Exproblab_Assignment_1/blob/master/robot_state_msgs/action/ComputePath.action) and [FollowPath](https://github.com/IlMusu/Exproblab_Assignment_1/blob/master/robot_state_msgs/action/FollowPath.action).

### ROS Parameters
The <b>ontology_map_builder</b> node uses the following parameters:
- `/ontology_reference (string)` : The reference name of the ontology.
- `/ontology_path (string)` : The global path of the default ontology.
- `/ontology_uri (string)` : The uri of the ontology.
- `/map (list)` : The list of pairs (room, [doors]) for building the map.
- `/room_positions (list)` : The list of pairs (room, position) for the rooms positions.
- `/robot_room (string)` : The initial room at which the robot is located.

The <b>robot_behaviour</b> node uses the following parameters:
- `/urgency_threshold (int)` : The urgency threshold time in seconds for the robot policy
- `/battery_require_recharge (int)` : The battery level under which it is necessary to start recharging.
- `/battery_stop_recharge (int)` : The battery level over which it is possible to stop recharging.
- `/recharge_rooms (list)` : The list of rooms containing a recharge station.
- `/rooms_priority (list)` : The list of room classes in order of priority.
- `/exploration_seconds (int)` : The time in seconds the robot takes to explore a room.

The <b>battery_controller</b> node uses the following parameters:
- `/battery_initial_value (int)` : The initial value for the battery level.
- `/execution_mode (string)` : The execution mode of this controller (MANUAL or RANDOM).

The <b>planner_controller</b> node uses the following parameters:
- `/execution_mode (string)` : The execution mode of this controller (MANUAL or RANDOM).

The <b>motion_controller</b> node uses the following parameters:
- `/execution_mode (string)` : The execution mode of this controller (MANUAL or RANDOM).

## 5. RUNNING CODE
### Waiting For The Ontology

<p align="center">
![](https://raw.githubusercontent.com/IlMusu/Exproblab_Assignment_1/documentation/gifs/wait_for_ontology.gif?token=GHSAT0AAAAAAB3G3VIRHNZ74P77UA5Q3H72Y4EUCRA)
</p>

The gif shows two running nodes:
- On the right there is the `robot_behavior` node.  
- On the left there is the `ontology_map_building` node.  
The <b>robot_behavior node</b> requests to the <b>ontology_map_building</b> node the reference name used to load the ontology into ARMOR. This value is provided by the second node only once it has completed loading and modifying the ontology.

### Moving In The Environment
<p align="center">
![](https://raw.githubusercontent.com/IlMusu/Exproblab_Assignment_1/documentation/gifs/moving.gif?token=GHSAT0AAAAAAB3G3VIQMR5UH6ELW5OTTED4Y4EUFWQ)
</p>

The gif show four running nodes:  
- On left there is the `robot_behavior` node.  
- On the right, from top to bottom, there are:  
  - The `battery_controller` node executed in MANUAL mode.  
  - The `planner_controller` node executed in MANUAL mode.  
  - The `movement_controller` node executed in MANUAL mode.  

It can be seen that at first the <b>robot_behavior</b> makes a request to the <b>/compute_path</b> ActionServer in order to obtain a path that goes from its current position to the position of room R4.  
Since the <b>planner_controller</b> node is executed in MANUAL mode, the user has to prompt the path in console in order to make the node finalize the <b>/compute_path</b> ActionServer response.  
Then, <b>robot_behavior</b> makes a request to the <b>/follow_path</b> ActionServer in order to make the robot follow the previously computed path.  
Again, since the <b>movement_controller</b> node is executed in MANUAL mode, the user has to prompt the command in console in order to make the robot move and finalize the <b>/follow_path</b> ActionServer response.  
At the end, in the <b>robot_behavior</b> console it is shown that the robot reaches the room R4.  



## 4. LIMITATIONS, FEATURES AND FUTURE WORK
### Limitations
The surveillance software has been developed under the following hypothesis:
- The robot in a fixed two-dimensional environment that does not change in time.
- The robot is always able to compute and follow a path to the goal position.
- Even if the battery is completely empty, the robot is still able to reach the recharge station.

### Features
The developed software provides the following features:
- The level of the battery of the robot may become low at any time.
- The movement policy makes it possible to abstracts the map of the environment.
- The creation of different maps is easy, fast and intuitive.
- The policy for choosing the next room may be changed at run-time easily.
- Abstraction from the path planning and movement procedures.
