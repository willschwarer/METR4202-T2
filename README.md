# METR4202 Project - Team 2

## Running the project

### Inverse Kinematics

Firstly make sure roscore is running. This can be done by running `roscore` in 
a terminal and leave it opened.

In another terminal you will need to start the dynamixel interface. this can be 
done using the following commands

```shell
cd ~/catkin_ws
source devel/setup.sh
roslaunch dynamixel_interface dynamixel_interface_controller.launch
```

In another terminal you will then to start the inverse kinematics node using 
the following

```shell
cd ~/catkin_ws
source devel/setup.sh
roslaunch metr4202_project inverse_kinematics_ROS.py
```

Cube locations can then be published using the following command
```shell
rostopic pub /cube_location geometry_msgs/Point32 "x: 0.0
y: 0.0
z: 0.0" 
```
