#include "denso_gazebo_control/denso_robot_hw_sim.h"
#include <sstream>
#include <algorithm>
#include <string.h>
#include <XmlRpcValue.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <pluginlib/class_list_macros.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace denso_gazebo_control
{
DensoRobotHWSim::DensoRobotHWSim() : robot_name_(""), robot_joint_num_(0), arm_joint_num_(0), gripper_joint_num_(0)
{
  joint_types_.clear();
  joint_cmd_.clear();
  joint_pos_.clear();
  joint_vel_.clear();
  joint_eff_.clear();
  joint_names_.clear();
  pid_.clear();
  sim_joints_.clear();
}

DensoRobotHWSim::~DensoRobotHWSim()
{
}

bool DensoRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle nh, gazebo::physics::ModelPtr model,
                              const urdf::Model* const urdf_model,
                              std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  sim_joints_ = model->GetJoints();

  if (!nh.getParam("robot_name", robot_name_))
  {
    ROS_WARN_STREAM("Failed to get robot_name parameter.");
  }

  std::string arm_joints_param = "/" + robot_name_ + "/arm_controller/joints";
  XmlRpc::XmlRpcValue arm_joints_list;

  if (!nh.getParam(arm_joints_param, arm_joints_list))
  {
    ROS_WARN_STREAM("Failed to get " << arm_joints_param << " parameter.");
  }
  else
  {
    arm_joint_num_ = arm_joints_list.size();
    for (int i = 0; i < arm_joint_num_; i++)
    {
      joint_names_.push_back(arm_joints_list[i]);
    }
    std::sort(joint_names_.begin(), joint_names_.end());
  }

  std::string gripper_joints_param = "/" + robot_name_ + "/gripper_controller/joints";
  XmlRpc::XmlRpcValue gripper_joints_list;

  if (!nh.getParam(gripper_joints_param, gripper_joints_list))
  {
    ROS_WARN_STREAM("Failed to get " << gripper_joints_param << " parameter.");
  }
  else
  {
    gripper_joint_num_ = gripper_joints_list.size();
    for (int i = 0; i < gripper_joint_num_; i++)
    {
      joint_names_.push_back(gripper_joints_list[i]);
    }
  }

  robot_joint_num_ = arm_joint_num_ + gripper_joint_num_;

  joint_types_.resize(robot_joint_num_);
  joint_cmd_.resize(robot_joint_num_);
  joint_pos_.resize(robot_joint_num_);
  joint_vel_.resize(robot_joint_num_);
  joint_eff_.resize(robot_joint_num_);

  pid_.resize(robot_joint_num_);

  for (int i = 0; i < robot_joint_num_; i++)
  {
    if (!nh.getParam(joint_names_[i], joint_types_[i]))
    {
      ROS_WARN_STREAM("Failde to get joint type: " << joint_names_[i]);
    }
  }

  for (int i = 0; i < robot_joint_num_; i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names_[i]), &joint_cmd_[i]);
    position_joint_interface_.registerHandle(pos_handle);
  }

  int arm_group = 0;
  if (!nh.getParam("arm_group", arm_group))
  {
    ROS_INFO("Use arm group 0.");
    arm_group = 0;
  }

  int gripper_group = 0;
  if (!nh.getParam("gripper_group", gripper_group))
  {
    ROS_INFO("Use gripper group 0.");
    gripper_group = 0;
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  std::vector<std::string> cmd_handle_names = position_joint_interface_.getNames();
  for (size_t i = 0; i < robot_joint_num_; i++)
  {
    const std::string name = cmd_handle_names[i];
    hardware_interface::JointHandle cmd_handle = position_joint_interface_.getHandle(name);

    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(name);
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;
    if (!getJointLimits(urdf_joint, limits) || !getSoftJointLimits(urdf_joint, soft_limits))
    {
      ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
    }
    else
    {
      joint_limits_interface_.registerHandle(
          joint_limits_interface::PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

      ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
    }
  }

  for (size_t i = 0; i < robot_joint_num_; i++)
  {
    ros::NodeHandle joint_nh(nh, "/gazebo_ros_control/pid_gains/" + joint_names_[i]);

    if (!pid_[i].init(joint_nh))
    {
      continue;
    }
  }

  return true;
}

void DensoRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  for (size_t i = 0; i < robot_joint_num_; i++)
  {
    if (joint_types_[i] == 0)
    {
      joint_pos_[i] = sim_joints_[i]->GetAngle(0).Radian();
    }
    else
    {
      joint_pos_[i] += angles::shortest_angular_distance(joint_pos_[i], sim_joints_[i + 1]->GetAngle(0).Radian());
    }
    joint_vel_[i] = sim_joints_[i + 1]->GetVelocity(0);
    joint_eff_[i] = sim_joints_[i + 1]->GetForce(0);
  }
}

void DensoRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  joint_limits_interface_.enforceLimits(period);

  for (size_t i = 0; i < robot_joint_num_; i++)
  {
    sim_joints_[i + 1]->SetPosition(0, joint_cmd_[i]);
  }
}
}  // namespace denso_gazebo_control

PLUGINLIB_EXPORT_CLASS(denso_gazebo_control::DensoRobotHWSim, gazebo_ros_control::RobotHWSim)
