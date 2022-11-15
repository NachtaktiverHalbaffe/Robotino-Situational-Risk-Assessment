#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_manager");  // Init ROS
  //;                         // Construct class
  ros::spin();                               // Run until interupted
  return 0;
}
