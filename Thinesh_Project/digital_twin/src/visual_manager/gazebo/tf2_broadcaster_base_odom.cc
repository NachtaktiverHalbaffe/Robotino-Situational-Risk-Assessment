// for documentation see the tutorial of tf2
// http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
// https://github.com/ros/geometry_tutorials/blob/indigo-devel/turtle_tf2/src/turtle_tf2_broadcaster.cpp

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_broadcaster_base_odom");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("odom", 5, &poseCallback);

  ros::spin();
  return 0;
};