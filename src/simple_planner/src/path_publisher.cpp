#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 10);

    ros::Rate rate(1);  // 1 Hz

    while (ros::ok())
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";

        
        for (double i = 0; i <= 5.0; i += 0.5)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = i;
            pose.pose.position.y = i;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;  // nessuna rotazione
            path.poses.push_back(pose);
        }

        path_pub.publish(path);
        ROS_INFO("Published dummy path with %ld poses", path.poses.size());

        rate.sleep();
    }

    return 0;
}

 

