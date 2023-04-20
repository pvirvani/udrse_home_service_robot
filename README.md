<!-- This file contains the metapackages and packages for home service robot as the final project for Udacity Nanodegree program -->
# Home Service Robot - Udacity-NDRSE-Project

## This file contains the details of Udacity's Nanodegree Program *`` Robotics Software Engineer ''* (including used packages) for home service robot as the final project.

---


## **Project Goals**

The goal of this project was to design a robot's environment in gazebo and program the home-service robot that will map it's environment and autonomously navigate to pre-specified pickup and drop-off locations. For this, one needed to:

* Design robot's environment with the Building Editor in Gazebo.
* Teleoperate the robot and manually test SLAM.
* Use the ROS navigation stack and manually command the robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
* Write a pick_objects node that commands the robot to move to the desired pickup and drop off zones.
* Write an add_markers node that subscribes to the robot odometry and publishes pick-up and drop-off markers to rviz.
* modify pick_objects node and add_markers node to establish communication between them, to complete desired home service robot implementation

### Prerequisites
(Note: The project was written and performed on the system with following provided configuration. It is also possible to run it on Ubuntu 16.04/18.04 with ROS Kinetic/Melodic)


* Ubuntu 20.04 OS with default make (4.2.1) and g++/gcc (9.4.0) packages, Boost Libraries (1.71.0)
    * chech boost libraries version with:
    ``` 
        dpkg -s libboost-dev | grep 'Version'
    ```
* ROS Noetic with Gazebo = 9.0
* the following ROS packages were used and the process of obtaining them is detailed in the link:
	* [gmapping](http://wiki.ros.org/gmapping)
	* [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
	* [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
	* [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

     or can be directly downloaded from the github.
    ```
    git clone https://github.com/ros-perception/slam_gmapping
    git clone https://github.com/turtlebot/turtlebot
    git clone https://github.com/turtlebot/turtlebot_interactions
    git clone https://github.com/turtlebot/turtlebot_simulator
    ```



### Directory Tree and contents (as suggested by the task)

```
.
├── README.md
├── CMakeLists.txt
├── add_markers
│   ├── include
│   │   └── ...
│   └── src
│       ├── add_markers_pk.cpp
│       └── add_markers_test_pk.cpp
│   ├──  ... ...
├── map
│   ├── Simple.world
│   ├── SimpleMap.pgm
│   ├── SimpleMap.yaml
├── pick_objects
│   └── src
│       ├── pick_objects_pk.cpp
│       └── pick_objects_test_pk.cpp
│   ├──  ... ...
├── rvizConfig
│   └── default.rviz
├── scripts
│   ├── add_marker.sh
│   ├── home_service.sh
│   ├── pick_objects.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── slam_gmapping
│   ├── gmapping
│   |── ... ...
├── turtlebot
│   |── turtlebot_teleop
│   |── ... ...
├── turtlebot_interactions
│   |── turtlebot_rviz_launchers
│   |── ... ...
|── turtlebot_simulator
│   ├── turtlebot_gazebo
│   |── ... ...
|── ... ...
```

This directory represents the main project's `src` folder structure with following contents

* README.md: this file.
* **add_markers** - add marker C++ node
* **map** - map and gazebo world files
* **pick_objects** - pick-objects C++ node
* **rvizConfig** - folder with rViz configurations used with some launch scripts
* **scripts** - shell scripts
	* `add_marker.sh` - script for testing add_marker concept with `add_markers_test_pk.cpp`
	* `home_service.sh` - main script for the home-service-robot
	* `pick_objects.sh` - script for testing pick_objects concept with `pick_objects_test_pk.cpp`
	* `test_navigation.sh` - script for testing navigation
	* `test_slam.sh` - script for performing SLAM and preparing map
* **slam_gmapping** - official ROS package with `gmapping_demo.launch` file
* **turtlebot** - official ROS package with `keyboard_teleop.launch` file
* **turtlebot_interactions** - official ROS package with `view_navigation.launch` file
* **turtlebot_simulator** - official ROS package with `turtlebot_world.launch` file

---

### Clone and Build

Since the folder presented here comprises only of ROS package, one needs to first create a catkin workspace and initialize it. Also, note that the official ROS packaged are already included here, but their dependencies need to be installed; steps for this are given below.

Within your `home` directory, execute the following:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Within `~/catkin_ws/src/` download or clone folders of this repository:

* **`Note` : the link is not valid yet ... it will be once the project is accepted completely**.
```
cd ~/catkin_ws/src/

git clone https://github.com/pvirvani/home-service-robot.git
```

Install dependencies:

```
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
```

`NOTE`: If any of the official packages give error, I recommed you delete associated folder and clone with src folder using appropriate line from here:

```
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git
```

Go back to catkin workspace and build it

```
cd ~/catkin_ws/
catkin_make
```

### Launch specific application and visualize

Specific applications can be launched using scripts provided. In this section I will go over how I have used these scripts.

##### Testing
First of all test the ROS official packages that you have installed. To test them write the shell scripts. In this project, all the shell scripts are written in `/.../src/scripts`

For SLAM-test go to `/.../src/scripts` folder and run `test_slam.sh` script:

```
cd ~/catkin_ws/src/scripts

chmod +x test_slam.sh  [comment]: # (to make the individual/specific shell script executable)

chmod +x *.sh  [comment]: # (to make all shell scripts executable)

./test_slam.sh
```

This will launch:

* `turtlebot_world.launch` to deploy turtlebot in my world with specific pose
* `gmapping_demo.launch` to perform SLAM
* `view_navigation.launch` to observe map in rviz
* `keyboard_teleop.launch` to manually control robot


**`Note` : It is to be noted that you might have some `problem` in the `turtlebot_world.launch` due to `enviroment variables` if you are using the `official ROS package`. This environment variable  `problem can be solved` by `providing the whole path` of your world file either in the `terminal, shell script, or in the turtlebot_world.launch file`. In `my project`, I have `solved the problem` by `modifying the turtlebot_world.launch` file.**

##### Navigation test
To test the navigation in Rviz environmnet, use `test_navigation.sh` file. This will launch:

* `turtlebot_world.launch` to deploy turtlebot in my world with specific pose
* `amcl_demo.launch` to localize turtlebot
* `view_navigation.launch` to observe map in rviz

Rviz's `2D Nav` tab is used to manually point out to two different goals, one at a time, and direct the robot to reach them and orient itself with respect to them.


##### Navigation Goal Node (pick-objects)

To test robot's capability to reach multiple goals, as specified by the program (and not manually), I created pick_objects package and specifically `pick_objects_test_pk.cpp` function. This can be tested following script which launches turtlebot, AMCL, rviz and pick_objects node:

```
cd ~/catkin_ws/src/scripts

chmod +x pick_objects.sh  [comment]: # (to make the individual/specific shell script executable)

./pick_objects.sh
```

##### Virtual Objects Node (add-markers)

To model a virtual object with markers in rviz, I created add_markers package and specifically `add_markers_test_pk.cpp` function. This can be tested following script which launches turtlebot, AMCL, rviz and add_markers node:

```
cd ~/catkin_ws/src/scripts

chmod +x add_markers.sh  [comment]: # (to make the individual/specific shell script executable)

./add_markers.sh
```
A sampe output is shown here:


##### Home-Service-Robot package

To simulate a full home service robot capable of navigating to pick up and deliver virtual objects, communication was established between the add_markers and pick_objects nodes via a `"/visualization_marker"` topic. For this purpose modified versions of previous test codes were created respectively `pick_objects_pk.cpp` and `add_markers_pk.cpp`. The entire package can be launched using:

```
cd ~/catkin_ws/src/scripts

chmod +x home_service.sh

./home_service.sh
```
