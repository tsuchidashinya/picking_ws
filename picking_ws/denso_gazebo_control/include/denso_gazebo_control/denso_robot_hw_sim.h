#ifndef DENSO_ROBOT_HW_SIM_H
#define DENSO_ROBOT_HW_SIM_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <gazebo_ros_control/robot_hw_sim.h>  // for simulator
#include <control_toolbox/pid.h>              // for simulator

#include <vector>
#include <string>

#include <gazebo/gazebo.hh>           // for simulator
#include <gazebo/physics/physics.hh>  // for simulator
#include <gazebo/common/common.hh>    // for simulator

namespace denso_gazebo_control
{
class DensoRobotHWSim : public gazebo_ros_control::RobotHWSim
{
public:
  DensoRobotHWSim();
  virtual ~DensoRobotHWSim();

  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions);

  ros::Time getTime() const
  {
    return ros::Time::now();
  }

  ros::Duration getPeriod() const
  {
    return ros::Duration(0.008);
  }

  void readSim(ros::Time, ros::Duration);
  void writeSim(ros::Time, ros::Duration);

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  std::vector<int> joint_types_;
  std::vector<double> joint_cmd_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<std::string> joint_names_;

  std::string robot_name_;
  int robot_joint_num_;
  int arm_joint_num_;
  int gripper_joint_num_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;                                // for simulator
  std::vector<control_toolbox::Pid> pid_;                                            // for simulator
  joint_limits_interface::PositionJointSoftLimitsInterface joint_limits_interface_;  // for simulator
};
}  // namespace denso_gazebo_control

#endif  // DENSO_ROBOT_HW_SIM_H
