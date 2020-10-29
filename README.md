# Exp_rob_lab_ass1
Assignment 1 of experimental robotics

Introduction
--------------

This is the Assignment 1 of Experimental Robotics Laboratory.
The code simulates a "dog" robot which walks randomly and sleeps until the user inputs a "play" command.
The play command can be input at any times and the robot will finish its movement and go to play.

Software Architecture and States Diagrams
----------------------------------------

For the software architecture:
![] (https://github.com/angux94/Exp_rob_lab_ass1/blob/main/Architecture_ass1.png)

As seen on the image, the architecture of the program has 4 nodes.
  - command_recog: Waits for the user to input the "play" command.
  - gesture_recog: Once the play command has been introduced, displays a 1000x1000 grid to select the play location.
  - movement_control: Controls the robot movement and tells us whe it arrives to the destination.
  - sm_assignment: The state machine of the system which control what the robot does.
  
For the State Machine Diagram:
![] (https://github.com/angux94/Exp_rob_lab_ass1/blob/main/State_Machine_diagram.png)

The State Machine has 3 states:
  - NORMAL: Main state in which the robot walks randomly until the "play" command arrives and goes to PLAY state, if not, goes to SLEEP state.
  - SLEEP: The robot goes to the sleep position and rests for a while, afterwars wakes up and goes to the NORMAL state.
  - PLAY: The robot plays for a number of times, going back and forth between the man and the specified play coordintate. Then goes to NORMAL state.
  
Messages
----------

The message types used in the project were:
  - geometry_msgs Point: for x,y coordinates of the robot.
  - std_msgs String: for strings.
  - std_msgs Bool: for booleans.
  
Parameters
-------------

The parameter are to be specified just on the sm_assignment file:
  - man_x: Man x coordinate (between 0-1000) (Default 400)
  - man_y: Man y coordinate (between 0-1000) (Default 600)
  - play_times: amount of times the robot is going to play (between 1-5) (Default random)
  
  - sleep_x: Bed x coordinate (between 0-1000) (Default 100)
  - sleep_y: Bed y coordinate (between 0-1000) (Default 900)
  - time_sleep: Time the robot sleeps (between 1-10) (Default random)
  
Packages and File list
------------

-Nodes:
All the executable files are in /exp_rob_lab_ass1/src folder which are the .py files for each node.

-The documentation can be found in the folder : /docs
Where in /builds/html can be found the html links for each of the nodes documentation.

Intstallation and Running Procedure
-----------------

As for visualization purposes, the project does not contain a launch file, as we need multiple tabs to interact with the nodes.
By downloading the provided ROS package, you can run each of the nodes on 4 terminals.

-For the command_recog node:
  rosrun exp_rob_lab_ass1 command_recog.py

  Will run the code and waits for a valid command.


-For the gesture_recog node:
  rosrun exp_rob_lab_ass1 geture_recog.py
  
  Will wait to display the grid when the command is entered.


-For the movement_control node:
  rosrun exp_rob_lab_ass1 movement_control.py
  
  Will wait to receive a coordinate and tell when it arrives and to which corrdinates.
  
  
-For the sm_assignment node:
  rosrun exp_rob_lab_ass1 sm_assignment.py
  
  Will run the state machine, in this we can specify the parameters of the system like:
    rosrun exp_rob_lab_ass1 sm_assignment.py _man_x:= 300 _man_y:= 900 _play_times:= 4 ....... (Depends on the parameters specified)

All nodes are to be executed simultaneously and wait for them to initialize.

Working Hypothesys and Environment
-------------

For the working hypothesys the robot should start in a normal state, for which will move randomly until the play command arrives or the amount of times finishes and goes to sleep. To afterwards re enter the normal behavior. If a play command is sent during the random walks, the robot will finish its current walk and then enter the play state even if he hasn't finished all the random walks. If the robot is sleeping, he must wake up first in order to enter the play state, if a command is sent during the sleeping time, he will wake up, go to normal state and directly to play state.

The environment is a 1000x1000 grid, with a man, a bed and the dog. Man and bed positions can be specified. The robot will tell to which coordinates it arrives in each movement.

System's Features
------------
  - Terminal to input commands and any time.
  - Terminal which shows the reached coordinates.
  - 1000x1000 display to select play location on the grid.
  - Terminal which tells the state, and the action that is being performed.
  - The play action can be sent at any time during the behavior of the robot, and it will go to play state as soon as the current movement is finished.
  
System's Limitations and technical improvements
------------

No launch file due to multiple terminals needed.

For future work the launch file could be generated if found a way to display multiple terminals at onece (couldn't implement that solution)

Author and Contacts
------
Andres Gutierrez
email: andresgut94@gmail.com

EMARO+ master in Advanced Robotics







  
