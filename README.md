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

## Ontology
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
- a LOCATION is thing that has the <b>visitedAt</b> property.
  - a CORRIDOR is a LOCATION that has at least 2 <b>hasDoor</b> properties.
  - a ROOM is a LOCATION that has at least 2 <b>hasDoor</b> properties.
  - a URGENT is a LOCATION.
- a ROBOT is a thing that has the <b>now</b>, <b>isIn</b>, <b>urgencyThreshold</b> properties.
  
The <b>visitedAt</b> data property contains the last time in seconds at which the robot visited the thing.  
The <b>hasDoor</b> object property contains the name of a DOOR.  
The <b>now</b> data property contains the value of the current time in seconds  
The <b>isIn</b> object property contains the name of a LOCATION.  
The <b>urgencyThreshold</b> data property of contains a delta time value in seconds.  
  
The following pseudo-code explains when a LOCATION becomes URGENT to be visited by the ROBOT:  

	 ROBOT.now - LOCATION.visitedAt > ROBOT.urgencyThreshold


## Environment
The indoor environment considered in this assignment is the following one:
- The ROOMs are <b>R1</b>, <b>R2</b>, <b>R3</b>, <b>R4</b>.
- The CORRIDORs are <b>E</b>, <b>C1</b>, <b>C2</b>.
- The LOCATION <b>E</b> contains the recharging station.
- The LOCATION <b>E</b> is the one in which the robot is positioned.

<p align="center">
	<img src="https://i.imgur.com/SQZ4ySu.png" />
</p>

## Surveillance Policy
In order to explore the environement, the robot must follow a <b>surveillance policy</b> which is fully described by the following rules which are sorted for greater priority:
1. The robot should go to the E location when the battery is low.
2. The robot should go into any reachable URGENT locations.
3. The robot prefers to stay in CORRIDORS.

## 2. SOFTWARE ARCHITECTURE

