#ifndef MODELVELOCITY_H
#define MODELVELOCITY_H
#include <math.h>

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <iostream>
#include <thread>

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/TwistStamped.h>

namespace gazebo
{
class ModelVelocity : public ModelPlugin
{
public:
  /// \brief Called once at the beginning of the simulation, gazebo needs this
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  /// \brief Called by the world update start event, "main-function"
  void OnUpdate();
  /// \brief Handle an incoming message from ROS
  void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg);
  /// \brief ROS helper function that processes messages
  void QueueThread();

private:
  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;
  /// \brief A ROS subscriber
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;
  /// \brief Pointer to the model
  physics::ModelPtr model;
  /// \brief Pointer to the update event connection
  event::ConnectionPtr updateConnection;

};
}  // namespace gazebo


#endif // MODELVELOCITY_H
