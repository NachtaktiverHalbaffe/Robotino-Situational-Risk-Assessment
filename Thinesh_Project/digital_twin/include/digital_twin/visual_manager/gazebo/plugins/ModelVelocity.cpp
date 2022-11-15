#include "ModelVelocity.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelVelocity)

// Called once at the beginning of the simulation
void ModelVelocity::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Store the pointer to the model
  model = _parent;

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelVelocity::OnUpdate, this));

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create the ROS node.
  rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      "/cmd_vel", 1000, boost::bind(&ModelVelocity::OnRosMsg, this, _1), ros::VoidPtr(), &rosQueue);
  
  rosSub = rosNode->subscribe(so);

  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&ModelVelocity::QueueThread, this));
}

// Called by the world update start event
void ModelVelocity::OnUpdate()
{
  //ROS_INFO("ModelVelocity::OnUpdate()");
}

// Handle an incoming message from ROS
void ModelVelocity::OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
{
  //ROS_INFO("ModelVe
  float xx = _msg->linear.x;
  ROS_INFO("Filtered velocities:%f,%f,%f,%f,%f,%f",_msg->angular.x, _msg->angular.y, _msg->angular.z ,_msg->linear.x,  _msg->linear.y,  _msg->linear.y);
  model->SetAngularVel(ignition::math::Vector3d( _msg->angular.x, _msg->angular.y, _msg->angular.z ));
  model->SetLinearVel(ignition::math::Vector3d( _msg->linear.x,  _msg->linear.y,  _msg->linear.z));
}

// ROS helper function that processes messages
void ModelVelocity::QueueThread()
{
  static const double timeout = 0.01;
  while (rosNode->ok())
  {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}