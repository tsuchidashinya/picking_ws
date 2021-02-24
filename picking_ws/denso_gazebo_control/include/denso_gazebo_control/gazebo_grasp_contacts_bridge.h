#ifndef GAZEBO_GRASP_CONTACTS_BRIDGE_H
#define GAZEBO_GRASP_CONTACTS_BRIDGE_H

#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <nav_msgs/Odometry.h>
#include <string>

namespace gazebo_grasp_contacts_bridge
{
class GazeboGraspContactsBridge
{
public:
  GazeboGraspContactsBridge(ros::NodeHandle& nh);
  ~GazeboGraspContactsBridge();

private:
  void forcesCb(ConstContactsPtr& msg);
  void positionCb(const nav_msgs::Odometry::ConstPtr& msg2);

private:
  ros::NodeHandle nh_;
  bool airborne_;
  std::string robot_name_;
  ros::Publisher contacts_pub_;
  ros::Subscriber position_sub_;
  gazebo::transport::SubscriberPtr contacts_sub_;
  gazebo::transport::NodePtr node_;
};
}  // gazebo_grasp_contacts_bridge

#endif  // GAZEBO_GRASP_CONTACTS_BRIDGE_H