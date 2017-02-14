# EE4308 Turtlebot Project

## Installation
1. Copy this folder into the `src/` repository of the catkin workspace.
2. Make the Python scripts executable with the command: 
    * `chmod +x ee4308_turtlebot_project/src/*.py`
3. Build the package with `catkin_make`.

## Execution
* __Part1, known map__: `roslaunch ee4308_turtlebot_project part1.launch`

* __Part 2, unknown map__: `roslaunch ee4308_turtlebot_project part2.launch`

## Using a new world
1. Copy the `.world` file in the  `worlds/` directory of the package.
2. Modify the first line of the launch file `part1.launch` or `part2.launch` with:
    * `<arg name="world_name" default="$(find ee4308_turtlebot_project)/worlds/<world_file>”/>`
3. If the corresponding map should be known by the robot at startup (part 1), update the `config.py` file in `src/` by adding the coordinates of the walls (excluding map boundaries).