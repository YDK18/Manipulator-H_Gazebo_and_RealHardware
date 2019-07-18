# Aecobot instruction
### I. Repository instruction
This repository is for combine husky platform(from clearpath) and manipulator-h (from robotis).

- Software:
ROS version : kinetic

- Hardware:
(intel realsense d435, husky from clearpath, manipulator-H from robotis)

### II. Contents
1. husky package
- Gmapping for SLAM(AMCL pakage is included in gmapping for localization)
- move_base package for path planning

2. metapackages for manipualtor-H
- robotis' manipulator-H control with moveit!

### III. Github basic command
> git init

> git add -A

> git commit -m "string here"

> git remote add origin xxxxxxxxxxxxxxx.git

> git push -u origin master

after connect local and github repository, just use below commands for sync.
> git pull

> git push

If you want to change your branch, you have to enter this commend 

> git checkout <branch>

If we want to download a specified version of this repository, then run with below command. If we use git clone to download repository, then the default version we download is master. So...we have to download proper version that pkg needs. The example is like V.errors. 1st error.. Terrible.

> git clone https://github.com/Fred159/aecobot.git --branch aecobot-dev

### IV. Publish topic to control the manipulator-H
#### 1. Publish goal position in the task space.
Manipulator support GUI environment for controlling manipulator-H.
If we want to control the manipualtor-H with code. 
Then, we run below format code.
TIP: The spacebar position is important

> rostopic pub /robotis/base/kinematics_pose_msg manipulator_h_base_module_msgs/KinematicsPose '{pose: {position: {x: 0.372, y: 0.191, z: 0.426}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'

#### 2. Control the husky movement with keyboard
install method
> sudo apt-get install ros-kinetic-teleop-twist-keyboard

run teleop keyboard file.
> rosrun teleop_twist_keyboard teleop_twist_keyboard.py

[Teleop keyboard tutorial page](http://wiki.ros.org/teleop_twist_keyboard)

-------------------------------------------
### V. Errors when install solution and things we should know
**Errors and solution**
1. The realsense plugin should insatll gazebo 7.0 version . If we got a error like 'no SimTime(), Name() error in realsense gazebo plugin , then we should change code as this link.'. If someont meets this error, please download the proper realsense-gazebo plugin with branch gazebo7. 
> Change the Simtime() to GetSimTime(), Name() to GetName() in gazebo-realsense/gzrs/RealSensePlugin.cc[link](https://github.com/SyrianSpock/realsense_gazebo_plugin/issues/20)

2. When install the velodyne simulator for gazebo, if error message shows like' cant find sdf/sdf.h'
> install the sdfformat from source

3. If error message shows like 'gazebo ros config ', re install below pkg.
> sudo apt-get install ros-kinetic-gazebo-ros

4. If error likes " package can't find qt build" , then
> sudo apt-get install ros-kinetic-qt-ros

5. If rqt can't open with comman, then run 
> rqt_xxxxxxxxx --force-discover

6. If velodyne_simulator catkin_make with error..... (maybe below is not the solution, but it works)
> format the computer, then it can build properly.

7. If position controller can't load in manipulator_h, then we should install pkg below.
Don't forget change the version in tutorial (indigo -> kinetic).[link](https://github.com/qboticslabs/mastering_ros/issues/7)

> sudo apt-get install ros-kinetic-joint-state-controller : This will install joint_state_controller package

> sudo apt-get install ros-kinetic-effort-controllers : This will install Effort controller

> sudo apt-get install ros-kinetic-position-controllers

8. If we can't find catkin build command, then we can follow this link
> [link](https://catkin-tools.readthedocs.io/en/latest/installing.html)

9. IF error like 'Could not find a package configuration file provided by "moveit_msgs" '.
> sudo apt-get install ros-kinetic-moveit

10. if " CMake Error at /opt/ros/hydro/share/catkin/cmake/catkinConfig.cmake:75 (find_package):
  Could not find a configuration file for package cmake_modules.

  Set cmake_modules_DIR to the directory containing a CMake configuration
  file for cmake_modules.  The file will have one of the following names:

    cmake_modulesConfig.cmake
    cmake_modules-config.cmake"
    
> sudo apt-get install ros-kinetic-cmake-modules


11. if error like " can't infd tf2-sensor-msgs"
> sudo apt-get install ros-kinetic-tf2-sensor-msgs


**Things we should know**
1. we have to add below path to gazebo-realsense/CMakeLists.txt for install realsense gazebo plugin when we reference the goven tutorial.( the pkg here, we added already.)

> set ( GZRS_PLUGIN_INSTALL_PATH /usr/lib/x86_64-linux-gnu/gazebo-7/plugins/ )

> set ( GZRS_MODEL_INSTALL_PATH ~/.gazebo/models )

2. Add a ar marker tutorial. follow this link
[ar marker link](https://www.youtube.com/watch?v=WDhIaVOUwsk)
[ar_marker_tracker](https://www.youtube.com/watch?v=8aQGe18eGOw)

--------------------------------------------
### VI. Log
(update contents should be write in the above of the previous text)
20190630 Current state
1. Add velodyne link and assign the sensor plugin to the link.(finished)
2. Velodyne pointcloud_to_laser node is generated and confirm the performance in the rviz.Check the pointcloud2 and check the velodyne/laser scan

20190621 markdown update By M.L.
main markdown file update.

20190621 by Yoon.D.K.
github  test.

-----------------------------------------------
### VII. To do list
(update contents should be write in the above of the previous text.)
20190630 update by M.L.
1. add realsense d435 plugin and test in rviz
2. assign the link with plugin
3. make it easy to switch different kinds of sensors. By referencing the other files structure, follow the structure kinetic camera 'false', 'true' format.
4. make a rviz file to open default gui that includes laser2d, pointcloud, depth camera's image and depth information.
5. make a integral launch file that can directly control the husky and slam.
6. upload the file's' in github or webhard disk.




