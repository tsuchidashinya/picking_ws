#include <iostream>
#include <map>
#include <std_msgs/String.h>
#include <string.h>

#include <denso_gazebo_srvs/Pose.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

static const double MAX_SPEED = 5.0;
static const double MIN_SPEED = 0.1;

geometry_msgs::Pose arm_pose_initialized()
{
  geometry_msgs::Pose initial_pose;
  initial_pose.orientation.w = -0.000152221;
  initial_pose.orientation.x = 1.0;
  initial_pose.orientation.y = -9.82654e-07;
  initial_pose.orientation.z = -0.000537402;
  initial_pose.position.x = 0.28727;
  initial_pose.position.y = 4.20769e-06;
  initial_pose.position.z = 0.604027;

  return initial_pose;
}
geometry_msgs::Pose arm_pose_place()
{
  geometry_msgs::Pose initial_pose;
  initial_pose.orientation.w = 0.0;
  initial_pose.orientation.x = 0.70710678118;
  // initial_pose.orientation.x = -0.70710678118;
  initial_pose.orientation.y = 0.70710678118;
  initial_pose.orientation.z = 0.0;
  initial_pose.position.x = 0.0028727;
  initial_pose.position.y = 0.5;
  // initial_pose.position.y = -0.5;
  initial_pose.position.z = 0.304027;

  return initial_pose;
}

geometry_msgs::Pose arm_pose_place2()
{
  geometry_msgs::Pose initial_pose;
  initial_pose.orientation.w = -0.000152221;
  initial_pose.orientation.x = 1.0;
  initial_pose.orientation.y = -9.82654e-07;
  initial_pose.orientation.z = -0.000537402;
  initial_pose.position.x = 0.6727;
  initial_pose.position.y = 4.20769e-06;
  initial_pose.position.z = 0.304027;

  return initial_pose;
}

geometry_msgs::Pose set_arm_pose(const geometry_msgs::TransformStamped& transform)
{
  geometry_msgs::Pose set_pose;
  set_pose.orientation.w = transform.transform.rotation.w;
  set_pose.orientation.x = transform.transform.rotation.x;
  set_pose.orientation.y = transform.transform.rotation.y;
  set_pose.orientation.z = transform.transform.rotation.z;
  set_pose.position.x = transform.transform.translation.x;
  set_pose.position.y = transform.transform.translation.y;
  set_pose.position.z = transform.transform.translation.z;

  return set_pose;
}

moveit::planning_interface::MoveGroupInterface::Plan
scaling_execution_speed(const double& speed_scale, const moveit::planning_interface::MoveGroupInterface::Plan& initial_plan)
{
  moveit_msgs::RobotTrajectory initial_trajectory;
  moveit_msgs::RobotTrajectory new_trajectory;

  initial_trajectory = initial_plan.trajectory_;
  new_trajectory = initial_trajectory;

  int n_joints = initial_trajectory.joint_trajectory.joint_names.size();
  int n_points = initial_trajectory.joint_trajectory.points.size();

  std::cout << "[scaling_execution_speed] speed_scale: " << speed_scale << std::endl;
  std::cout << "[scaling_execution_speed] n_points: " << n_points << std::endl;

  for (int i = 1; i < n_points; i++)
  {
    ros::Duration start_time(initial_trajectory.joint_trajectory.points[i].time_from_start.toSec() / speed_scale);
    new_trajectory.joint_trajectory.points[i].time_from_start = start_time;

    for (int j = 0; j < n_joints; j++)
    {
      new_trajectory.joint_trajectory.points[i].velocities[j] =
          initial_trajectory.joint_trajectory.points[i].velocities[j] * speed_scale;
      new_trajectory.joint_trajectory.points[i].accelerations[j] =
          initial_trajectory.joint_trajectory.points[i].accelerations[j] * speed_scale * speed_scale;
      new_trajectory.joint_trajectory.points[i].positions[j] =
          initial_trajectory.joint_trajectory.points[i].positions[j];
    }
  } 
  

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;
  new_plan = initial_plan;
  new_plan.trajectory_ = new_trajectory;
  return new_plan;
}

bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& group, const geometry_msgs::Pose& target,
                      const double& speed_scale)
{
  group.setStartState(*group.getCurrentState());

  robot_state::RobotState robot_state_start(*group.getCurrentState());
  std::map<std::string, double> joints;
  joints["joint_1"] = robot_state_start.getVariablePosition("joint_1");
  joints["joint_2"] = robot_state_start.getVariablePosition("joint_2");
  joints["joint_3"] = robot_state_start.getVariablePosition("joint_3");
  joints["joint_4"] = robot_state_start.getVariablePosition("joint_4");
  joints["joint_5"] = robot_state_start.getVariablePosition("joint_5");
  joints["joint_6"] = robot_state_start.getVariablePosition("joint_6");

  std::cout << "=====Current Joint Angle=====" << std::endl;
  std::cout << "joint_1: " << joints["joint_1"] << std::endl;
  std::cout << "joint_2: " << joints["joint_2"] << std::endl;
  std::cout << "joint_3: " << joints["joint_3"] << std::endl;
  std::cout << "joint_4: " << joints["joint_4"] << std::endl;
  std::cout << "joint_5: " << joints["joint_5"] << std::endl;
  std::cout << "joint_6: " << joints["joint_6"] << std::endl;
  std::cout << "\n" << std::endl;

  group.setPoseTarget(target);

  robot_state::RobotState robot_state_goal(*group.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

  bool ik = robot_state_goal.setFromIK(joint_model_group, target, 4, 1);
  std::cout << "bool ik = " << ik << std::endl;

  joints["joint_1"] = robot_state_goal.getVariablePosition("joint_1");
  joints["joint_2"] = robot_state_goal.getVariablePosition("joint_2");
  joints["joint_3"] = robot_state_goal.getVariablePosition("joint_3");
  joints["joint_4"] = robot_state_goal.getVariablePosition("joint_4");
  joints["joint_5"] = robot_state_goal.getVariablePosition("joint_5");
  joints["joint_6"] = robot_state_goal.getVariablePosition("joint_6");
  group.setJointValueTarget(joints);

  std::cout << "=====Joint Angle=====" << std::endl;
  std::cout << "joint_1: " << joints["joint_1"] << std::endl;
  std::cout << "joint_2: " << joints["joint_2"] << std::endl;
  std::cout << "joint_3: " << joints["joint_3"] << std::endl;
  std::cout << "joint_4: " << joints["joint_4"] << std::endl;
  std::cout << "joint_5: " << joints["joint_5"] << std::endl;
  std::cout << "joint_6: " << joints["joint_6"] << std::endl;
  std::cout << "\n" << std::endl;

  moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
  group.plan(initial_plan);

  moveit::planning_interface::MoveGroupInterface::Plan result_plan = scaling_execution_speed(speed_scale, initial_plan);

  group.execute(result_plan);

  return ik;
}

bool move_permission;
void Callback(const std_msgs::String::ConstPtr& msg)
{
  std_msgs::String buf = *msg;
  if (strcmp(buf.data.c_str(), "true") == 0)
  {
    move_permission = true;
  }
  else
  {
    move_permission = false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_and_grasp");
  ros::NodeHandle nh;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/vs087/grasp_position", 1000);
  ros::Subscriber sub = nh.subscribe("/vs087/move_permission", 1000, Callback);

  geometry_msgs::Pose target;
  geometry_msgs::TransformStamped transform;
  moveit::planning_interface::MoveGroupInterface group("arm");

  double speed_scale;

  ros::param::param<double>("~speed_scale", speed_scale, 1.0);
  if (speed_scale > MAX_SPEED)
  {
    speed_scale = MAX_SPEED;
    ROS_WARN_STREAM("speed_scale limit from " << MIN_SPEED << " to " << MAX_SPEED << " !!");
  }
  else if (speed_scale < MIN_SPEED)
  {
    speed_scale = MIN_SPEED;
    ROS_WARN_STREAM("speed_scale limit from " << MIN_SPEED << " to " << MAX_SPEED << " !!");
  }

  ros::ServiceClient client = nh.serviceClient<denso_gazebo_srvs::Pose>("/vs087/grasp");
  ros::ServiceClient rclient = nh.serviceClient<denso_gazebo_srvs::Pose>("/vs087/release");
  denso_gazebo_srvs::Pose srv;

  //初期位置
  target = arm_pose_initialized();
  pub.publish(target);
  plan_and_execute(group, target, MAX_SPEED);

  while (ros::ok())
  {
    // grasp_point位置
    while (ros::ok())
    {
      try
      {
        transform = tfBuffer.lookupTransform("base_link", "grasp_point", ros::Time(0));
        std::cout << "successes lookup " << std::endl;
        break;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        ROS_WARN("Waiting grasp_point tf");
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    //取りに行く
    target = set_arm_pose(transform);
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);
    //掴む
    if (!client.call(srv))
    {
      ROS_INFO("Failed grasp");
    }
    else
    {
      ROS_INFO("Success grasp");
    }
    //置く
    target = arm_pose_place();
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);
    //離す
    if (!rclient.call(srv))
    {
      ROS_INFO("Failed release");
    }
    else
    {
      ROS_INFO("Success release");
    }
    sleep(2);
    //初期位置
    target = arm_pose_initialized();
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);
    ros::spinOnce();

    // 2回目
    while (ros::ok())
    {
      try
      {
        transform = tfBuffer.lookupTransform("base_link", "grasp_point", ros::Time(0));
        std::cout << "successes lookup " << std::endl;
        break;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        ROS_WARN("Waiting grasp_point tf");
        ros::Duration(1.0).sleep();
        continue;
      }
    }
    //ぶつかるので上の方に移動する
    target = arm_pose_place();
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);
    target = set_arm_pose(transform);
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);

    if (!client.call(srv))
    {
      ROS_INFO("Failed grasp");
    }
    else
    {
      ROS_INFO("Success grasp");
    }
    target = arm_pose_place2();
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);

    if (!rclient.call(srv))
    {
      ROS_INFO("Failed release");
    }
    else
    {
      ROS_INFO("Success release");
    }
    sleep(2);
    target = arm_pose_initialized();
    pub.publish(target);
    while (!move_permission)
    {
      ros::spinOnce();
    }
    plan_and_execute(group, target, speed_scale);
    ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}