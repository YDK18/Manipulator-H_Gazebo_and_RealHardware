# Execute Manipulator-H  in gazebo.

### I. Repository instruction
This repository is for manipulator-h (from robotis).

- Software:
ROS version : kinetic

- Hardware:
(manipulator-H)

### II. Execute gazebo

> roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch

### III. Execute Manipulation

> roslaunch manipulator_h_manager manipulation.launch

# gazebo and real-hardware work together.

### I. Leave the starting gazebo

### II. Execute dynamixel_ctrl package

This connects the dynamixel.

> roslaunch aeco_manipulator_h_dynamixel_ctrl dynamixel_ctrl.launch

### III. Execute interface package

The interface package connects moveit! package and dynamixel package.

> roslaunch aecobot_manipulator_h_interface aecobot_manipulator_h_interface.launch

# If you want to execute only real-hardware, you just execute aeco_manipulator_h package.


### I. Execute dynamixel_ctrl package

This connects the dynamixel.

> roslaunch aeco_manipulator_h_dynamixel_ctrl dynamixel_ctrl.launch

### I. Execute Moveit! package

> roslaunch aecobot_manipulator_h_moveit demo.launch

### III. Execute interface package

The interface package connects moveit! package and dynamixel package.

> roslaunch aecobot_manipulator_h_interface aecobot_manipulator_h_interface.launch
