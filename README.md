## ResearchTrack-I-Nonholonomic Robot

**Farzin Sarvari (S5057724)**

**Final assignment of Research Track 1**
---
## How to running
we are going to run ROS package nonholonomic_controlling by using 'gmapping' and 'final_assignment'(required by the CMake file).
If all these three packages are present on the machine it's sufficient to run:
```bash
# catkin_make
```
It is necessary for the simulation to run the simulation_gmapping.launch file present inside'final_assignment' package in another shell window
to run both **rviz** and **Gazebo**.
```bash
# roslaunch final_assignment simulation_gmapping.launch

# roslaunch nonholo_control nonholo_control.launch

# roslaunch nonholo_control user_interface.launch
```

## Content description
The content of the package is the following:

**CMakeLists.txt:** the cmake file of the package

**map_library.h:** header file for functions

**nonholonomic_controlling.launch:** one of the two launch files regarding to the package, contains the definition of the nodes that relate to the part of computation.

**user_interface.launch:** the other launch file, it is defined the nodes that interface with the user and will be run separately from the one and printing on screen the position of the robot.

**package.xml:** the XML file describing package requisites.

**nonholo_params.yaml:** parameter file including variables shared between all the nodes.

**bug_restart_service.py:** python script that invokes roslaunch API to launch.

**gotopo_wllflw_redirect.cpp:** a ServiceServer that forwards service calls for 'go_to_point_switch' and 'wall_follower_switch', directed in the launch file.

**map_library.cpp:** the executable for the library.

**random_position_server.cpp:** a ServiceServer generating a random target position between the valid ones.

**robot_mainframe.cpp:** the central node, it brings all tasks performed by other nodes.

**target_reached_detection.cpp:** a message Publisher that sends a message each time when target is received.

**unreachable_target_detection.cpp:** when the robot is stuck and cannot reach its defined target, publishing a message on a specific topic in that case.

**user_interface.cpp:** representing the list of commands that can be issued to the robot.

**user_position_server.cpp:** similar to random_position_server.cpp, the position here is asked to the user.

**TargetPos.srv:** a service structure defining the target position of robot.

**UIMenu.srv:** a service structure defining the user choice inserted in the UI.

---
## Robot behaviour
Our robot, can move around the environment, going towards one of the 6 targets coordinates allowed, using either
**'move_base'** or **'bug0'** path planning algorithm, accepting new commands user any time when reach the goal. Moreover, it can start following the walls of the environment without a fixed goal. It is also that, due to the limited capabilities of the **'bug0' algorithm**, it's possible that the robot get stuck in one of the "rooms" in the simulationed environment, following the walls without ever actually managing to move towards the goal. To address that case, however, a simple recovery
behaviour has been implemented, having the robot fall back to the previous target using the **'move_base'** algorithm in case more than some minutes pass from the definition of a target and its achievement. While the robot is trying to reach a target, distance between the two are printed on screen by tf_echo.cpp that enabling the tracking of the system.

---

**Please notice that this is a quick description, a more explanation is included inside each script.**
