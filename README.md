## ResearchTrack-I-Final-assignment
---
**Farzin Sarvari (S5057724)**

**Final assignment of Research Track 1**
---
## how to running
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
---
## Robot behaviour
Our robot, can move around the environment, going towards one of the 6 targets coordinates allowed, using either
**'move_base'** or **'bug0'** path planning algorithm, accepting new commands user any time when reach the goal. Moreover, it can start following the walls of the environment without a fixed goal. It is also that, due to the limited capabilities of the **'bug0' algorithm**, it's possibile that the robot get stuck in one of the "rooms" in the simulationed environment, following the walls without ever actually managing to move towards the goal. To address that case, however, a simple recovery
behaviour has been implemented, having the robot fall back to the previous target using the **'move_base'** algorithm in case more than some minutes pass from the definition of a target and its achievement. While the robot is trying to reach a target, distance between the two are printed on screen enabling the tracking of the system.

---

**Please notice that this is a quick description, a more explanation is included inside each script.**
