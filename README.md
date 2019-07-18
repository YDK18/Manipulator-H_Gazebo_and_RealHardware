# Start Manipulator-H  in gazebo.

### I. Repository instruction
This repository is for manipulator-h (from robotis).

- Software:
ROS version : kinetic

- Hardware:
(manipulator-H)

### II. Start gazebo

> roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch

### III. Start Manipulation

> roslaunch manipulator_h_manager manipulation.launch

# gazebo and real-hardware work together.

### I. Leave the starting gazebo

### II. Start dynamixel_ctrl package

This connects the dynamixel.

> roslaunch aeco_manipulator_h_dynamixel_ctrl dynamixel_ctrl.launch

### III. Start interface package

The interface package connects moveit! package and dynamixel package.

> roslaunch aecobot_manipulator_h_interface aecobot_manipulator_h_interface.launch
