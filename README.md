Assignment 2 - Race Robot
===============================

This is the second assignment of the course Research Track 1, provided by Università Degli Studi di Genova, Robotics Engineering degree.
The simulation includes a robot, placed inside a race track (Monza Circuit), equipped with a laser scanner. The robot must drive autonomously around the circuit. While doing so, he must avoid collisions, change its velocity, and reset its position depending on the user's choice.
The simulation environment is the following:
![simulation_environment](https://github.com/FraFerrazzi/second_assignment/blob/main/world/tracciato.png)
This solution is developed by: Francesco Ferrazzi 5262829.

Table of Contents
----------------------

- [Assignment 2 - Race Robot](#assignment-2---race-robot)
  * [Table of Contents](#table-of-contents)
  * [Installing and running](#installing-and-running)
  * [Exercise Description](#exercise-description)
  * [controllerrobot_node](#controllerrobot-node)
    + [get_min_val](#get-min-val)
    + [decide_direction](#decide-direction)
    + [set_velocity](#set-velocity)
  * [ui_node](#ui-node)
    + [print_ui](#print-ui)
    + [check_user_input](#check-user-input)
  * [Nodes Relationship Flowchart and Pseudocode](#nodes-relationship-flowchart-and-pseudocode)
  * [Pseudocode](#pseudocode)
    + [controllerrobot_node](#controllerrobot-node)
    + [ui_node](#ui-node)
  * [Possible Improvements](#possible-improvements)

Installing and running
----------------------

The simulator requires a [ROS Noetic](http://wiki.ros.org/noetic/Installation) installation.

To run the program is sufficient to clone this repository in the `src` folder placed inside your ROS workspace and type on the terminal: 
```bash
$ roslaunch second_assignment second_ass.launch
```

If the program does not work, make sure to compile it with the command:
```bash
$ catkin_make
```
Be careful: `catkin_make` should be typed in the root of your ROS workspace.


Exercise Description
-----------------------------

The objective of the assignment is to make the robot go around the race circuit. Meanwhile, the robot must avoid collisions with boundaries, allow the user to set its velocity, and reset its position.
To implement the solution, two nodes were developed:
* `controllerrobot_node`
* `ui_node`

The `controllerrobot_node` works with `stageros` node. The communication between the two occurs using topics such as the `/base_scan` topic which is used to acquire the laser scan data from the world.
Another topic used in the controller node is the `/cmd_vel` topic that, through a publisher, defines the velocity of the robot.
In this node a service is used to allow the communication between the `controllerrobot_node` and the `ui_node`. The message is received from the controller through the `/set_velocity` topic using the second_assignment::VelService.

The `ui_node` is done to handle input given by user. When a command arrives, it checks that is correct. According to the input, the command can be sent to the `controllerrobot_node` using the service previously described, or can be handled by the `/reset_positions` topic that resets the robot position. 

Since I wrote in the first assignment some possible improvements, such as going faster or keeping the robot as centered as possible, I tried to develop this behavior in the project.


## controllerrobot_node ##

The `controllerrobot_node` guarantees the assumption of the robot's correct behavior while moving around the race track. It was necessary to implement the following functions to achieve the goal:
* `get_min_val`
* `decide_direction`
* `set_velocity`

### get_min_val ###

The function `get_min_val` allows getting the minimum value among the elements of a given array. The whole vector is acquired thanks to the laser scan of the robot. The vector is subsequently divided into five regions of space. The aim is to allow the robot to understand where the closest obstacle is placed. 

### decide_direction ###

This function is executed each time something is published in the base_scan topic. The function aims to acquire the closest obstacle in each region of space in front of the robot calling the `get_min_val` function and choosing how the robot should behave to avoid any collision. The objective of `decide_direction` is also to make the robot stay as centered as possible in the circuit in every instant. This works fairly well until the robot moves with a linear velocity that is not too high.

### set_velocity ###

This function executes each time the user publishes a command on the topic of the service. Based on the user decision, the velocity (both angular and linear) of the robot changes increasing or decreasing. The updated speed is printed on the terminal as feedback for the user. 

## ui_node ##

The user interface node prints on screen the commands that the user can choose, with the `print_ui` function, and waits until input from the keyboard is given by the user. The `ui_node` keeps running until `ros::ok()` returns true and the key decided from the user is not `9`. If the input of the user is `0`, the robot resets its position. If the command is `1` through `6`, the `set_velocity` service is called.

### print_ui ###

Prints on the screen the commands that the user can choose from the keyboard.

### check_user_input ###

This function check if the user's choice is an integer number. If it is, the function return the integer number that will be managed by the `main` function. If not, a message is printed on the terminal and the program won't do anything until a correct input is given.

Nodes Relationship Flowchart and Pseudocode
-----------------------------

The relationship between the nodes is explained in the following graph:
![relathionship_between_nodes](https://github.com/FraFerrazzi/second_assignment/blob/main/images/nodes_rel_all.png)

## Pseudocode ##

### controllerrobot_node ###

```
while controllerrobot_node is running
	
	get distances from obstacles thanks to laser scanner
	split the ranges in five regions
	get minimum obstacle distance in each region
	
	if obstacles in all directions are far away and robot is centered in the lane
		go straight max linear velocity
	else 
		if robot too close to obstacle
			if obstacle on the right
				stop and turn left
			else
				stop and turn right
		else
			if robot is very close to right obstacle in respect to left one	
				turn left
				go straight with low speed
			else if robot is very close to left obstacle in respect to right one
				turn right
				go straight with low speed
			else if robot is closer to right obstacle in respect to left one	
				turn left with almost half speed
				go straight with half speed
			else if robot is closer to left obstacle in respect to right one
				turn right with almost half speed
				go straight with half speed
			else if robot almost centered but closer to right	
				turn left a bit
				go straight with almost max speed
			else if robot almost centered but closer to left
				turn right a bit
				go straight with almost max speed
			else if robot almost centered but slightly closer to right	
				turn slightly left 
				go straight max speed
			else if robot almost centered but slightly closer to left
				turn slightly right 
				go straight max speed
			else
				if obstacle on the right
					turn left
					go straight with low speed
				else
					turn left
					go straight with low speed
	
	listen for ui_node
	
	if ui_node sends a command
		if command == 1
			decrease linear and angular velocity a lot
		if command == 2
			decrease linear and angular velocity 
		if command == 3
			decrease linear and angular velocity a little
		if command == 4
			increase linear and angular velocity a little
		if command == 5
			increase linear and angular velocity 
		if command == 6
			increase linear and angular velocity a lot
		if command == 9
			exit node		
```

### ui_node ###

```
while ui_node is running
	
	print user interface
	
	waits for user input
	check that the input is integer number
	while user input != from integer number
		ask new user input
	
	if user input == 0
		reset robot position
	if user input != 0 
		send command to controllerrobot_node
```

Possible Improvements
-----------------------------

There are four main possible future improvements that I came up with, which are:
* Make the robot move more fluently, especially at higher velocity.
* Allow the user to manually control the robot movement as if it was a videogame.
* Let the robot reach `linear_velocity = 15` without crushing into any obstacle.
* Define a well written log file that the user can use to check what happens in the program in every instant, including possible errors.
