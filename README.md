# Turtlebot navigation stack

## Scope
We propose a generic scheme for the indoor navigation of an autonomous robotics system in both known and unknown environments, allowing it to efficiently and safely move between two arbitrary points. The stack, implemented as a ROS package, is validated using Gazebo simulations of a Turtlebot in an intricate room, modelled as a maze.

<p align="center">
	<img src="doc/plots/path_known_map.png” width="750"/>
</p>

## Overview

<p align="center">
	<img src="doc/plots/git_doc/graph_overview.png” width="750"/>
</p>

* **Map prior**: what is known beforehand (*a priori*) of the environment
* **Kinect**: a depth camera, returns a point cloud
* **Odometry**: the position of the robot in the maze
* **Map updater**: process the data to build and update an internal map of the environment
* **Global planner**: compute a path from start to goal using the map
* **Local planner**: ensure that the robot actually follows the path, computes the velocity commands

## Additional Features
* Real-time map building and visualisation using RViz
* User interaction such as goal definition through Rviz

## Installation
The package has been successfully tested with `ROS Indigo` on `Ubuntu 14.04`.

1. Copy this folder into the `src/` repository of the catkin workspace.
2. Make the Python scripts executable with the command: 
    * `chmod +x turtlebot_simple_navigation/src/*.py`
3. Build the package with `catkin_make`.

## Usage
* __Known map__: `roslaunch turtlebot_simple_navigation part1.launch`

* __Unknown map__: `roslaunch turtlebot_simple_navigation part2.launch`

# What comes next
* More efficient and natural path: PRM* for the nodes generation instead of the predefined grid
* Smoother control signal: spline as the reference input
* More robust wall detection: RANSAC plane fitting using the PointCloud Library

# Credits
The project was developed by Preben Jensen Hoel and Paul-Edouard Sarlin, both exchange students at the National University of Singapore for the academic year 2016-2017.
