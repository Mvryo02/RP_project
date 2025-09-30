#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	int free_cells = 0;
	int occupied_cells = 0;
	int unknown_cells = 0;
	for(auto cell : msg->data){
		if (cell==0) free_cells++;
		else if (cell==100) occupied_cells++;
		else unknown_cells++;
		
	}

	ROS_INFO("Map received: %d x %d", msg->info.width, msg->info.height);
	ROS_INFO("Free: %d, Occupied: %d, Unknown: %d", free_cells, occupied_cells, unknown_cells);
}

int main(int argc,  char** argv){

	ros::init(argc, argv, "map_listner");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/map", 1, mapCallback);

	ros::spin();

	return 0;
	
}

