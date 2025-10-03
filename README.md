# Robot Programming Exam Project
## **Assignment**
Write a program that computes the path to a user selected goal from the current location of the robot, received as the transform map->base_link. The program listens to a grid_map (similar tp wjat localizer
does), extracts the obstacles and the traversable surface (cells “birghter” than a value). The cost of being in a location depends on the distance to a closest obstacle (the smaller, the higher)  Using this cost function, the program computes the path (if existing) to the goal by using your favorite search algorithm. The goal pose is received from the /move_base/goal message
## **Project Description**
The project implements a simple path planner based on A* algorithm in **ROS Noetic** using **C++**.
The system is organized as a ROS package, called simple_planner, which contains the following programs:
- costmap.cpp/costmap.hpp, a class built from nav_msgs::OccupancyGrid and define a map with the costs for all the cells of the map, based on the distance from the closest obstacle, using Brushfire algorithm.
- planner.cpp/planner.hpp, given a starting cell and a goal cell defines a path using A* algorithm, with Manhattan distance heuristic.
- planner_node.cpp, main node which subscribes to /map, /move_base_simple/goal and /initialpose. Using a istance of the planner, it computes a path and publishes it on the topic /path.

Additional nodes used in the system:
- **Rviz** node subscribes to the topic /map and publishes the goal on /move_base_simple/goal and the initial poses on /initial_pose;
- tf_static_publisher which publishes on /tf_static the transformation from map to base_link
- map_server loads the environment map from a .yaml file and pubblishes it as a nav_msgs::OccupancyGrid on /map.
## Execution
To compile run "catkin_make" on RP_project fold.
To run "roslaunch simple_planner simple_planner.launch" and on **Rviz** select the initial position with "2D pose Estimate" and goal with "2D Nav Goal", then the planner computes the path and publishes it on /path, displayed in Rviz as a red line.
