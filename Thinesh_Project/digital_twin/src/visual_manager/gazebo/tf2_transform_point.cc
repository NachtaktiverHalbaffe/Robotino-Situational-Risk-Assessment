// for documentation see the tutorial of tf2
// http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2%3A%3AMessageFilter
// https://github.com/ros/geometry_tutorials/blob/indigo-devel/turtle_tf2/src/message_filter.cpp

#include "geometry_msgs/PointStamped.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

class PoseDrawer
{
public:
  PoseDrawer() : tf2_(buffer_), target_frame_("map"), tf2_filter_(point_sub_, buffer_, target_frame_, 1000, 0)
  {
    point_sub_.subscribe(n_, "/transform_point", 1000);
    pub1 = n_.advertise<geometry_msgs::PointStamped>("/transformed/camera", 1000);
    pub2 = n_.advertise<geometry_msgs::PointStamped>("/transformed/laser", 1000);
    pub3 = n_.advertise<geometry_msgs::PointStamped>("/transformed/range", 1000);
    tf2_filter_.registerCallback(boost::bind(&PoseDrawer::msgCallback, this, _1));
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const geometry_msgs::PointStampedConstPtr& point_ptr)
  {
    geometry_msgs::PointStamped point_out;

    try
    {
      buffer_.transform(*point_ptr, point_out, target_frame_);

      // ROS_INFO("transform: %s", point_ptr->header.frame_id.c_str());
      // ROS_INFO("transform: Position(x:%f y:%f z:%f)", point_out.point.x, point_out.point.y, point_out.point.z);

      // because geometry_msgs::PointsStamped cannot have additional data the differentiation of the messages is done
      // like this:
      if (point_ptr->point.x == 0 && abs(point_ptr->point.y) == 15)
        pub3.publish(point_out);
      else if (point_ptr->point.x == 15 && point_ptr->point.y == 0)
        pub3.publish(point_out);
      else if (point_ptr->header.frame_id.compare("camera") == 0)
        pub1.publish(point_out);
      else if (point_ptr->header.frame_id.compare("scan") == 0)
        pub2.publish(point_out);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Failure %s\n", ex.what());  // Print exception which was caught
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_drawer");  // Init ROS
  PoseDrawer pd;                         // Construct class
  ros::spin();                           // Run until interupted
  return 0;
};