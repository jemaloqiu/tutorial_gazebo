#include <algorithm>
#include <assert.h>
#include <stdlib.h> 
#include "wifibot_controller/wifibotController.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <fstream>

namespace gazebo
{

// Constructor
WifibotControllerPlugin::WifibotControllerPlugin()
{
}

// Destructor
WifibotControllerPlugin::~WifibotControllerPlugin()
{
  delete rosnode_, nh_;
  delete transform_broadcaster_;
}

// Load the controller
void WifibotControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();

  gzdbg << "plugin parent sensor name: " << parent->GetName() << "\n";

  if (!this->parent) 
	{ 
		gzthrow("WifibotController requires a Model as its parent"); 
	}

  this->robotNamespace = "Wifibot";
  
	if (_sdf->HasElement("robotNamespace"))	
	{
    		this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
  	}

  
	if (!_sdf->HasElement("topicName"))
	{
  		ROS_WARN("WifibotController plugin missing <topicName>, defaults to cmd_vel");
  		this->topicName = "cmd_vel";
	}
	else
	{
  		this->topicName = _sdf->GetElement("topicName")->GetValueString();
	}


  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "wifibot_controller_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);
  nh_ = new ros::NodeHandle("~");

  ROS_INFO("starting wifibot-controller");

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  sub_ =  rosnode_->subscribe(topicName, 1, &WifibotControllerPlugin::cmdVelCallback, this);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1); 
  joint_pub_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 1);

  time_not_init = true;

  math::Pose pose = this->parent->GetWorldPose();
  btQuaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  btVector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  robotPose[0] = pose.pos.x;
  robotPose[1] = pose.pos.y;
  robotPose[2] = qt.getAngle();
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  robotVel[0] = 0;
  robotVel[1] = 0;
  robotVel[2] = 0;

  odomVel[0] = 0;
  odomVel[1] = 0;
  odomVel[2] = 0;


  // start custom queue for wifibot_controller
  this->callback_queue_thread_ = boost::thread(boost::bind(&WifibotControllerPlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WifibotControllerPlugin::UpdateChild, this));
}


// Update the controller
//**********************************************
void WifibotControllerPlugin::UpdateChild()
{
  double dis, da;
  double stepTime = this->world->GetPhysicsEngine()->GetStepTime();
  // std::cout << "Tiem step of simulation: " <<  stepTime << "s" << std::endl;

  dis = stepTime * vel_lin;
  da = stepTime * vel_yaw; 


  // Compute odometric pose
  robotPose[0] += dis * cos(robotPose[2]);
  robotPose[1] += dis * sin(robotPose[2]);
  robotPose[2] += da;

  odomPose[0] += dis * cos(odomPose[2]);
  odomPose[1] += dis * sin(odomPose[2]);
  odomPose[2] += da;

  write_position_data();
  publish_odometry();
}

// Finalize the controller
void WifibotControllerPlugin::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}


void WifibotControllerPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  lock.lock();
  vel_lin = cmd_msg->linear.x;
  vel_yaw = cmd_msg->angular.z;
  lock.unlock();
} 

void WifibotControllerPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}


void WifibotControllerPlugin::publish_odometry()
{
  if (time_not_init) 
  {
	  initial_time = ros::Time::now();
	  time_not_init = false;
  }

  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  // getting data for base_footprint to odom transform
  math::Pose pose = this->parent->GetWorldPose();

  tf::Quaternion qt = tf::createQuaternionFromRPY(0, 0, odomPose[2]);
  tf::Vector3 vt( odomPose[0],  odomPose[1],  0.);

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                             current_time,
                                                             odom_frame,
                                                             base_footprint_frame));

  // publish odom topic
  odom_.pose.pose.position.x = odomPose[0];
  odom_.pose.pose.position.y = odomPose[1]; 

  odom_.pose.pose.orientation.x = qt[0]; 
  odom_.pose.pose.orientation.y = qt[1]; 
  odom_.pose.pose.orientation.z = qt[2]; 
  odom_.pose.pose.orientation.w = qt[3]; 

  
  odom_.twist.twist.linear.x = vel_lin*cos(odomPose[2]);
  odom_.twist.twist.linear.y = vel_lin*sin(odomPose[2]);
  odom_.twist.twist.angular.z = vel_yaw;

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  odom_.pose.covariance = boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3);
  odom_.twist.covariance = boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e3);

  pub_.publish(odom_);


    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(3);
    joint_state.position.resize(3);
    joint_state.name[0] ="translate_x";
    joint_state.position[0] = pose.pos.x;
    joint_state.name[1] ="translate_y";
    joint_state.position[1] = pose.pos.y;
    joint_state.name[2] ="rotation_yaw";
    joint_state.position[2] = robotPose[2];

  //joint_pub_.publish(joint_state);
}

void WifibotControllerPlugin::write_position_data()
{
  math::Pose orig_pose = this->parent->GetWorldPose();

  math::Pose new_pose = orig_pose;
  new_pose.pos.x = robotPose[0];
  new_pose.pos.y = robotPose[1];
  new_pose.rot.SetFromEuler(math::Vector3(0, 0, robotPose[2]));
  this->parent->SetWorldPose(new_pose);

}

GZ_REGISTER_MODEL_PLUGIN(WifibotControllerPlugin)
}

