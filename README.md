# METR4202 Team Project - Team 2

### Repository 

The Team 2 github repository was utilised for online accessable backups and effective version control.
SSH connections were set up for remote access to the Rasberry Pi, allowing for simultanious coding. 
Commits were pushed directly from the pi, and can be accessed from the link:

https://github.com/willschwarer/METR4202-T2

### A brief repository description 

The launch directory contains seperate .launch files:
    One launch file for tasks 1), 2), and 3a)
    One launch file for task 3b)

The scripts directory contains the python code for:
    Inverse kinematics solution
    Calculates modular and robust joint angle solutions for 4R robots of similar configurations from given x, y, z spacial coordinates.

    Block transforms
    Checks how close the cubes are from one another, and the distance and angle they are from the robot to ensure the cube is able to be picked up.

    State machines
    Implements the approved ros package SMACH to move between states.

    Gripper
    Provides funcitonality for opening and closing the gripper by a specified value.

## Running the project

### Tasks 1-3a

Firstly open up 3 terminals in terminator. In the first you will need to run `sudo pidpiod` and then `roscore`

In the next terminal you will need to run `roslaunch metr4204_project task_1_and_2.launch`.
When the first camera window opens, press space twice to change the camera into colour mode.

When both camera windows are opened in the final terminal run `rosrun metr4202_project state_machine.py`

### Tasks 3b

Firstly open up 3 terminals in terminator. In the first you will need to run `sudo pidpiod` and then `roscore`

In the next terminal you will need to run `roslaunch metr4204_project task_3b.launch`.
When the first camera window opens, press space twice to change the camera into colour mode.

When both camera windows are opened in the final terminal run `rosrun metr4202_project task_3b.py`

### Fun Task

Firstly open up 3 terminals in terminator. In the first you will need to run `sudo pidpiod` and then `roscore`

In the next terminal you will need to run `roslaunch metr4204_project task_1_and_2.launch`.
When the first camera window opens, press space twice to change the camera into colour mode.

When both camera windows are opened in the final terminal run `rosrun metr4202_project task_fun.py`
