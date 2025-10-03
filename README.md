# RP_project
Write a program that computes the path to a user selected goal from the current location of the robot, received as the transform map->base_link. The program listens to a grid_map (similar tp wjat localizer
does), extracts the obstacles and the traversable surface (cells “birghter” than a value). The cost of being in a location depends on the distance to a closest obstacle (the smaller, the higher)  Using this cost function, the program computes the path (if existing) to the goal by using your favorite search algorithm. The goal pose is received from the /move_base/goal message
