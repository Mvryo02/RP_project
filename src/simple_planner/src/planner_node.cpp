#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "simple_planner/costmap.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "simple_planner/planner.hpp"
#include <move_base_msgs/MoveBaseActionGoal.h>
using namespace std;
using namespace ros;
class PlannerNode{
public:
	PlannerNode(){
		ros::NodeHandle nh;
		//publisher
		path_pub_ = nh.advertise<nav_msgs::Path>("/path",1);
		//subscriber

		map_sub_ = nh.subscribe("/map", 1, &PlannerNode::mapCallback, this);
		pose_sub_ = nh.subscribe("initialpose", 1, &PlannerNode::initialPoseCallback, this);
		goal_sub_ = nh.subscribe("/move_base_simple/goal", 1 , &PlannerNode::goalCallback, this);
	}
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
		ROS_INFO("map received of dimension: %d, %d", msg->info.width, msg->info.height);
		costmap_.fromOccupancyGrid(*msg);
		map_frame_ = msg->header.frame_id;	
		resolution_ = msg->info.resolution;
		origin_x_ = msg->info.origin.position.x;
		origin_y_ = msg->info.origin.position.y;
		ROS_INFO("Map origin : (%.2f, %.2f), resolution: %.2f", origin_x_, origin_y_, resolution_);
	}
	
	void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
		start_x = (msg->pose.pose.position.x - origin_x_)/resolution_;
		start_y = (msg->pose.pose.position.y - origin_y_)/resolution_;
		has_start_ = true;
		ROS_INFO("Starting position received: (%d, %d)", start_x, start_y);
	}
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
		ROS_INFO("Goal received");
		if(map_frame_.empty()){
			ROS_WARN("No map received yet");
			return;
		}
		tf::StampedTransform transform;
		try{
			tf_listener_.lookupTransform(map_frame_, "base_link", ros::Time(0), transform);
		}catch(tf::TransformException &ex){
			ROS_WARN("TF lookup failure: %s", ex.what());
			return;
		}
		//int start_x = (transform.getOrigin().x() - origin_x_) / resolution_;
        	//int start_y = (transform.getOrigin().y() - origin_y_) / resolution_;
        	int goal_x = (msg->pose.position.x - origin_x_) / resolution_;
        	int goal_y = (msg->pose.position.y - origin_y_) / resolution_;
		int cost = costmap_.getCost(start_x,start_y);
		if(cost>=255)
			ROS_WARN("Starting cell is an obstacle");
		else
			ROS_INFO("Starting cell is free, with cost: %d", cost);
		int goal_cost = costmap_.getCost(goal_x, goal_y);
		if(goal_cost>=255)
			ROS_WARN("Goal cell is an obstacle");
		else
			ROS_INFO("Goal cell is free, with cost: %d", goal_cost);
        	ROS_INFO("Planning from (%d,%d) to (%d,%d)", start_x, start_y, goal_x, goal_y);

        	Planner planner(costmap_);
        	auto path_cells = planner.makePlan(start_x, start_y, goal_x, goal_y);

        	if (path_cells.empty()) {
            		ROS_WARN("No path found!");
            		return;
        	}

        
        	nav_msgs::Path path_msg;
        	path_msg.header.stamp = ros::Time::now();
        	path_msg.header.frame_id = map_frame_;

        	for (auto& cell : path_cells) {
            		geometry_msgs::PoseStamped pose;
            		pose.header = path_msg.header;
            		pose.pose.position.x = origin_x_ + cell.first * resolution_;
            		pose.pose.position.y = origin_y_ + cell.second * resolution_;
            		pose.pose.orientation.w = 1.0;
            		path_msg.poses.push_back(pose);
        	}

        	path_pub_.publish(path_msg);
        	ROS_INFO("Published path with %zu poses", path_msg.poses.size());
	}

private:
	Subscriber map_sub_;
	Subscriber goal_sub_;
	Subscriber pose_sub_;
	Publisher path_pub_;
	tf::TransformListener tf_listener_;
	Costmap costmap_;
	string map_frame_;
	double resolution_{0.5};
	double origin_x_{0.0};
	double origin_y_{0.0};
	int start_x;
	int start_y;
	bool has_start_;
};


int main(int argv, char** argc){
	ros::init(argv, argc, "planner_node");
	PlannerNode node;

	ros::spin();
	return 0;
}
