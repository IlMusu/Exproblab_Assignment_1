# EXPERIMENTAL ROBOTICS LABORATORY ASSIGNMENT 1
Author : Mattia Musumeci 4670261@studenti.unige.it  
  
This is the first assignment developed for the <b>Experimental Robotics Laboratory</b> course of the University of Genoa.  
At this <b>[link](http://documentaiton)</b> it is possible to find the documentation describing the software contained in this repository.  

## 1. INTRODUCTION
The scenario involves a <b>robot</b> deployed in an indoor environment for <b>surveillacnce purposes</b> whose objective is basically to visit the different environement locations and explore them for a given amount of time. In this context, the robot in equipped with a rechargable battery that, when not charged enough, needs to be recharge in specific location.  
  
The software contained in this repository has been developed for <b>[ROS Noetic 1.15.9](http://wiki.ros.org/noetic)</b>.  
The <b>Robot Operating System (ROS)</b> is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.  
  
The behaviour of the robot has been described with a finite state machine developed for <b>[SMACH](http://wiki.ros.org/smach)</b>.  
The <b>SMACH</b> library which is a task level architecture for describing and executing complex behaviours. At its core, it is a ROS-independent python library to build hierarchical state machines and the interaction with the ROS environment is obtained by implementing dedicated states inside the finite state machine.  
  
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
In order to explore the environement, the robot must follow a <b>surveillance policy</b> which is fully described by the following rules which are sorted for greater priority:
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
Notice that only one between the three shown commands are necessary to run the node. The last two commands show how it is possible to use command line arguments to modify the execution mode of the controllers ("RANDOM" or "MANUAL").
To see the complete list of arguments for this launch file, look at [this]() section.
```bash
roslaunch robot_behaviour robot_state.launch
# roslaunch robot_behaviour robot_state.launch move_controller_execution_mode:="RANDOM"
# roslaunch robot_behaviour robot_state.launch battery_controller_execution_mode:="RANDOM"
```

Finally, open a new console and run the robot_state node:
To see the complete list of arguments for this launch file, look at [this]() section.
```bash
roslaunch robot_behaviour robot_behaviour.launch
```


## 3. WORKING HYPOTHESIS
### Limitations
The surveillance software has been developed under the following hypothesis:
- The robot in an fixed two-dimensional environment that does not change in time.
- There is no concept of position for the robot: either the robot is or not is in a room.
- Even if the battery is complitely empty, the robot is still able to reach the recharge station.

### Features
The developed software provides the following features:
- The level of the battery of the robot may become low at any time.
- The robot movement between two rooms may not be completed fully.
- The movement policy makes it possible to abstracts the map of the environment.
- Abstraction from the path planning and movement procedures.

## 4. SOFTWARE ARCHITECTURE
### Component Diagram
In the <b>component diagram</b> are shown all the <b>blocks</b> and <b>interfaces</b> that have been used or developed in order to obtain the desired software architecture.

<p align="center">
	<img src="https://i.imgur.com/DoopjNY.png" />
</p>

- The `robot_behavior` node contains the state machine that describes the desired behavior of the robot. The functionality of this node is based on the interaction with other nodes. In particular, it interacts with:  
  - The `ARMOR` library through the <b>/armor_interface_srv</b> service.
  - The `BatteryStateController` subcomponent through the <b>/battery_level</b> message.
  - The `MoveStateController` subcomponent through the <b>/robot_move</b> action.
- The `robot_state` node is the representation of the current state of the robot and provides some functionalities to give to the user the possibility of changing the state of the robot. Basically, this node is container of subcomponents, each one describing a section of the state of the robot.  

A more detailed explaination of the use of the interfaces is available <b>[here](#ros-messages,-services-and-actions)</b>.

### States Diagram
This <b>state diagram</b> shows the state machine representing the desired behaviour of the robot. In particular, all the possible states and transitions are shown.

<p align="center">
	<img src="https://i.imgur.com/nynYen4.png" />
</p>

- Inside the `INITIALIZATION` state, the following operations are performed:  
  1) Retrieves parameters from the ROS Parameter Server.  
  2) Loads the requested ontology onto ARMOR.  
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

### ROS Messages, Services And Actions
In order to develop the interfaces between the components, the following messages have been used:  
- The <b>robot_behaviour</b> node which:  
  - Uses the <b>/armor_interface_srv</b> service, of type `ArmorDirective.srv`, to interact with the ARMOR in order to modify the ontology and retrieve knowledge.  
  - Subscribes to the <b>/battery_level</b> topic, of type `UInt8.msg`, to retrieve the updated battery level.  
  - Creates a client for the <b>/move_robot</b> action server, of type  ` MoveBetweenRooms.action` in order to move the robot from its current room to a goal room.  
- The <b>robot_state</b> node has:  
  - The <b>BatteryStateController</b> subcomponent which:  
    - Publishes on the <b>/battery_level</b> topic, of type `UInt8.msg`, the updated value of the battery level.  
  - The <b>MoveStateController</b> subcomponent which:  
    - Provides a server for the <b>/move_robot</b> action, of type  ` MoveBetweenRooms.action` in order to move the robot from its current room to a goal room.  

In more detail, the structure of the ` MoveBetweenRooms.action` is described in <b>[this](https://github.com/IlMusu/Exproblab_Assignment_1/blob/master/robot_state_msgs/action/MoveBetweenRooms.action)</b> file.
