#include <denso_gazebo_grasp/tercero_grasp.h>
#include <denso_gazebo_grasp/process_vector.h>

using tercero_grasp::TerceroGrasp;

TerceroGrasp::TerceroGrasp(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("Start Create Instance!!");
  ros::param::param<std::string>("~robot_name", robot_name_, "vs087");
  ros::param::param<int>("~angle_tolerance", angle_tolerance_, 100);
  ros::param::param<double>("~ratio_threshold", ratio_threshold_, 0.3);
  ros::param::param<double>("~time", time_, 0.01);
  ros::param::param<double>("/" + robot_name_ + "/joints_limit/gripper/finger_R_joint/min_position",
                            min_joint_position_, 0.0);
  ros::param::param<double>("/" + robot_name_ + "/joints_limit/gripper/finger_R_joint/max_position",
                            max_joint_position_, 0.06);
  grasp_force_sub_ = nh.subscribe("/" + robot_name_ + "/grasp_contacts", 1, &TerceroGrasp::graspForceCallback, this);
  get_chack_position_srv_ = nh.serviceClient<tercero_gazebo_srvs::GetJointsValue>("/tercero/get_chack_position");
  set_chack_position_srv_ = nh.serviceClient<tercero_gazebo_srvs::SetJointsValue>("/tercero/set_chack_position");
  grasp_server_ = nh.advertiseService("/" + robot_name_ + "/grasp", &TerceroGrasp::graspObject, this);
  release_server_ = nh.advertiseService("/" + robot_name_ + "/release", &TerceroGrasp::releaseObject, this);
  contact_points_.clear();
  ROS_INFO("Finish Create Instance!!");
}

void TerceroGrasp::graspForceCallback(const denso_gazebo_msgs::ContactArray::ConstPtr& msg)
{
  contact_points_ = msg->contacts;
}

bool TerceroGrasp::graspObject(denso_gazebo_srvs::Pose::Request& req, denso_gazebo_srvs::Pose::Response& res)
{
  get_chack_position_srv_.call(get_joints_srv_);
  double current_joint_value = get_joints_srv_.response.values[1];

  for (int i = 0;; ++i)
  {
    double joint_value = current_joint_value + (0.00005 * i);
    if (joint_value > max_joint_position_)
    {
      ROS_INFO_STREAM("Hand Full Close!!");
      break;
    }

    set_joints_srv_.request.value = joint_value;
    if (!set_chack_position_srv_.call(set_joints_srv_))
    {
      ROS_ERROR_STREAM("Error has occurred during hand set chack position");
      break;
    }

    std::vector<std::vector<double> > force_vectors;
    force_vectors.clear();
    if (contact_points_.size() > 1)
    {
      for (int j = 0; j < contact_points_.size(); j++)
      {
        std::vector<double> force_vector;

        force_vector.push_back(contact_points_[j].force[0]);
        force_vector.push_back(contact_points_[j].force[1]);
        force_vector.push_back(contact_points_[j].force[2]);

        force_vectors.push_back(force_vector);
      }
      for (int j = 0; j < force_vectors.size(); j++)
      {
        std::vector<double> force_vector_1 = force_vectors[j];
        for (int k = j + 1; k < force_vectors.size(); k++)
        {
          std::vector<double> force_vector_2 = force_vectors[k];
          double norm_1 = pv.getVectorNorm(force_vector_1);
          double norm_2 = pv.getVectorNorm(force_vector_2);

          if ((norm_1 < 1e-04) || (norm_2 < 1e-04))
          {
            continue;
          }

          double angle = pv.getDegFromVectors(force_vector_1, force_vector_2);

          if (angle > angle_tolerance_)
          {
            double ratio;
            if (norm_1 > norm_2)
            {
              ratio = norm_2 / norm_1;
            }
            else
            {
              ratio = norm_1 / norm_2;
            }

            if (ratio >= ratio_threshold_)
            {
              ROS_INFO_STREAM("Grasp Object!!");
              res.success = true;
              return res.success;
            }
            else
            {
              continue;
            }
          }
          else
          {
            continue;
          }
        }
      }
    }
    else
    {
      continue;
    }
    ros::Duration(time_).sleep();
  }

  res.success = false;
  return res.success;
}

bool TerceroGrasp::releaseObject(denso_gazebo_srvs::Pose::Request& req, denso_gazebo_srvs::Pose::Response& res)
{
  set_joints_srv_.request.value = min_joint_position_;
  if (!set_chack_position_srv_.call(set_joints_srv_))
  {
    ROS_ERROR_STREAM("Error has occurred during hand set chack position");
    res.success = false;
    return false;
  }
  ROS_INFO_STREAM("Release Objects!!");
  res.success = true;
  return true;
}
