# Robot Programming Exam Project
## **Assignment**
Write a program that computes the path to a user selected goal from the current location of the robot, received as the transform map->base_link. The program listens to a grid_map (similar tp wjat localizer
does), extracts the obstacles and the traversable surface (cells “birghter” than a value). The cost of being in a location depends on the distance to a closest obstacle (the smaller, the higher)  Using this cost function, the program computes the path (if existing) to the goal by using your favorite search algorithm. The goal pose is received from the /move_base/goal message
## **Project Description**
The project implements a simple path planner based on A* algorithm in **ROS Noetic** using **C++**.
The system is structured as a ROS package, called simple_planner, which contains the following programs:
- costmap.cpp/costmap.hpp, a class that is built based on a nav_msgs::OccupancyGrid and define a map with the cost of all the cells of the map, based on the distance from the closest obstacle.
- planner.cpp/planner.hpp, given a starting cell and a goal cell defines a path using A* algorithm, with Manhattan distance heuristic.
- planner_node.cpp, main node which subscribes to /map, /move_base_simple/goal and /initial_pose. Using a planner istances it evaluates a path and publishes it on the topic /path.

Then there are the nodes:
- Rviz node subscribes to the topic /map and publishes on /move_base_simple/goal and /initial_pose, respectivily the goal cell and the initial cell;
- tf_static_publisher which publishes on /tf_static the transformation from map to base_link
- map_server for the nav_msgs::OccupancyGrid map.
## Execution
To compile run "catkin_make" on RP_project fold, to run "roslaunch simple_planner simple_planner.launch" then select the initial position with "2D pose Estimate" and goal with "2D Nav Goal"
