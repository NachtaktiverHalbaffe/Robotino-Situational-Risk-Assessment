// for documentation see the tutorial of tf2
// http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
// https://github.com/ros/geometry_tutorials/blob/indigo-devel/turtle_tf2/src/turtle_tf2_broadcaster.cpp

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "robotino/odom";
  transformStamped.child_frame_id = "robotino/base";
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_broadcaster_base_odom");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("robotino/odom", 5, &poseCallback);

  ros::spin();
  return 0;
};