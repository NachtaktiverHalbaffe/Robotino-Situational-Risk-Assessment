#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

void callback(const nav_msgs::OdometryConstPtr& msg)
{
    //update dynamic env

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_env");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("robotino/obstacles", 5, &callback);

  ros::spin();
  return 0;
};