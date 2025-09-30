#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO("Goal received: x=%2.f, y=%.2f, frame_id:%s", msg->pose.position.x, msg->pose.position.y, msg->header.frame_id.c_str());

}


int main(int argv, char** argc)
{
	ros::init(argv, argc, "goal_listener");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);

	ros::spin();
	return 0;
}
