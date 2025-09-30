#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "simple_planner/costmap.hpp"


Costmap global_costmap;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	global_costmap.fromOccupancyGrid(*msg);
	global_costmap.computeDistanceCosts();
	ROS_INFO("Map received of size: %d, %d", global_costmap.getWidth(), global_costmap.getHeight());
	global_costmap.printDebug();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "test_costmap");
	ros::NodeHandle nh;

	ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);

	ros::spin();
	return 0;
}
