/*
 * This node sends periodically three points to tf2. The triangle of these three points represents the field of view of
 * the laser scanner. This is necessary for the module diagnosis_del_obs.
 *
 * author: Johannes Sigel
 */

#include <ros/ros.h>

#include "geometry_msgs/PointStamped.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_laser_range");
  ros::NodeHandle node;
  ros::Duration(1).sleep();
  ros::Publisher pub = node.advertise<geometry_msgs::PointStamped>("/transform_point", 3);
  geometry_msgs::PointStamped p1, p2, p3;
  p1.header.frame_id = p2.header.frame_id = p3.header.frame_id = "scan";
  p1.point.x = 0;
  p1.point.y = -15;
  p2.point.x = 15;
  p2.point.y = 0;
  p3.point.x = 0;
  p3.point.y = 15;
  ros::Rate loop_rate(2);  // 2 times per second
  while (ros::ok())
  {
    p1.header.stamp = ros::Time::now();
    p2.header.stamp = ros::Time::now();
    p3.header.stamp = ros::Time::now();
    pub.publish(p1);
    pub.publish(p2);
    pub.publish(p3);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}