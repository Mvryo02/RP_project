#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "fake_tf_broadcaster");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  ros::Rate rate(10.0);

  while (ros::ok()){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(1.0, 2.0, 0.0));  // posizione fittizia
    tf::Quaternion q;
    q.setRPY(0, 0, 1.57); // rotazione z = 90°
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    rate.sleep();
  }
  return 0;
}
