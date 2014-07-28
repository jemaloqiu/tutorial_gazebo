#ifndef WIFIBOT_CONTROLLER_PLUGIN_H
#define WIFIBOT_CONTROLLER_PLUGIN_H

#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <bullet/LinearMath/btQuaternion.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

extern "C"
{
	#include <stdlib.h>
}

namespace gazebo
{

class WifibotControllerPlugin : public ModelPlugin
{
  public: WifibotControllerPlugin();
  public: ~WifibotControllerPlugin();
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected: virtual void UpdateChild();
  protected: virtual void FiniChild();


private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  void write_position_data();
  void publish_odometry();
  void getPositionCmd();

  physics::WorldPtr world;
  physics::ModelPtr parent;
  physics::JointState joints;
  event::ConnectionPtr updateConnection;

  double vel_lin; // linear velocity -- speed
  double vel_yaw; // yaw anglular velocity
  double robotPose[3];
  double odomPose[3];
  double robotVel[3];
  double odomVel[3];

  physics::ModelState *robot_state_;
  std::vector<gazebo::physics::JointPtr>  joints_;
  physics::JointState js;

  // ROS STUFF
  ros::NodeHandle *rosnode_, *nh_;
  ros::Publisher pub_, joint_pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  sensor_msgs::JointState joint_state;
  std::string tf_prefix_;

  boost::mutex lock;

  std::string robotNamespace;
  std::string topicName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();
  ros::Time initial_time;
  // controller stuff
  bool alive_, time_not_init; 
};

}

#endif
