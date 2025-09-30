#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle nh;

  tf::TransformListener listener;

  ros::Rate rate(1.0);
  while (nh.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("map", "base_link", ros::Time(0), transform);
      ROS_INFO("Robot in map: x=%.2f, y=%.2f, yaw=%.2f",
               transform.getOrigin().x(),
               transform.getOrigin().y(),
               tf::getYaw(transform.getRotation()));
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
  return 0;
}
