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
  * [Installing and running](#installing-and-running)
  * [Exercise Description](#exercise-description)
  * [controller_node](#controller-node)
    + [get_min_val](#get-min-val)
    + [decide_direction](#decide-direction)
    + [set_velocity](#set-velocity)
  * [ui_node](#ui-node)
    + [print_ui](#print-ui)
  * [Pseudocode and Flowchart](#pseudocode-and-flowchart)
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

The objective of the assignment is to make the robot go around the race circuit. While doing so, the robot must avoid collisions with boundaries, allow the user to set its velocity, and reset its position.
To implement the solution, two nodes were developed:
* `controller_node`
* `ui_node`

Since I wrote in the first assignment some possible improvements, such as going faster or keeping the robot as centered as possible, I tried to develop this behavior in the project.


## controller_node ##

The controller_node guarantees the assumption of the robot's correct behavior while moving around the race track. It was necessary to implement the following functions to achieve the goal:
* `get_min_val`
* `decide_direction`
* `set_velocity`

### get_min_val ###

The function `get_min_val` allows getting the minimum value among the elements of a given array. The whole vector is acquired thanks to the laser scan of the robot. The vector is subsequently divided into five regions of space. The aim is to allow the robot to understand where the closest obstacle is placed. 

```

```

### decide_direction ###

This function is executed each time something is published in the base_scan topic. The function aims to acquire the closest obstacle in each region of space in front of the robot calling the `get_min_val` function and choosing how the robot should behave to avoid any collision. The objective of `decide_direction` is also to make the robot stay as centered as possible in the circuit in every instant. This works fairly well until the robot moves with a linear velocity that is not too high.

### set_velocity ###

This function executes each time the user publishes a command on the topic of the service. Based on the user decision, the velocity (both angular and linear) of the robot changes increasing or decreasing. The updated speed is printed on the terminal as feedback for the user. 

## ui_node ##

The user interface node prints on screen the commands that the user can choose, with the `print_ui` function, and waits until input from the keyboard is given by the user. The `ui_node` keeps running until `ros::ok()` returns true and the key decided from the user is not `9`. If the input of the user is `0`, the robot resets its position. If the command is `1` through `6`, the `set_velocity` service is called.

### print_ui ###

Prints on the screen the commands that the user can pick from the keyboard.

Nodes Relationship Flowchart
-----------------------------

The relationship between the nodes is explained in the following graph:
![relathionship_between_nodes](https://github.com/FraFerrazzi/second_assignment/blob/main/images/nodes%20relationship.png)

Possible Improvements
-----------------------------

There are three main possible improvements that I came up with, which are:
* Make the robot move more fluently, especially at higher velocity.
* Allow the user to manually control the robot movement as if it was a videogame.
* Let the robot reach `linear_velocity = 15` without crushing on any obstacle.

