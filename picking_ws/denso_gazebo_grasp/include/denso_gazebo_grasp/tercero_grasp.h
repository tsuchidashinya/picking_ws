#ifndef TERCERO_GRASP_H
#define TERCERO_GRASP_H

#include <denso_gazebo_grasp/process_vector.h>

#include <ros/ros.h>
#include <denso_gazebo_msgs/Contact.h>
#include <denso_gazebo_msgs/ContactArray.h>
#include <denso_gazebo_srvs/Pose.h>
#include <tercero_gazebo_srvs/GetJointsValue.h>
#include <tercero_gazebo_srvs/SetJointsValue.h>

namespace tercero_grasp
{
class TerceroGrasp
{
public:
  TerceroGrasp(ros::NodeHandle& nh);

  process_vector::ProcessVector pv;

private:
  void graspForceCallback(const denso_gazebo_msgs::ContactArray::ConstPtr& msg);
  bool graspObject(denso_gazebo_srvs::Pose::Request& req, denso_gazebo_srvs::Pose::Response& res);
  bool releaseObject(denso_gazebo_srvs::Pose::Request& req, denso_gazebo_srvs::Pose::Response& res);

private:
  ros::NodeHandle nh_;
  ros::Subscriber grasp_force_sub_;
  ros::ServiceServer grasp_server_;
  ros::ServiceServer release_server_;
  ros::ServiceClient get_chack_position_srv_;
  ros::ServiceClient set_chack_position_srv_;
  tercero_gazebo_srvs::GetJointsValue get_joints_srv_;
  tercero_gazebo_srvs::SetJointsValue set_joints_srv_;
  std::string robot_name_;
  int angle_tolerance_;
  double ratio_threshold_;
  double time_;
  double max_joint_position_;
  double min_joint_position_;
  std::vector<denso_gazebo_msgs::Contact> contact_points_;
};
}  // namespace tercero_grasp

#endif  // TERCERO_GRASP_H
