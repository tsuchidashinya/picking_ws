#include <denso_gazebo/model_tf_publisher.h>

#include <algorithm>

using model_tf_publisher::ModelTFPublisher;

ModelTFPublisher::ModelTFPublisher(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("Start Create Instance");
  ros::param::param<std::string>("~src_frame_name", src_frame_, "world");
  ros::param::param<std::string>("~model_name", urdf_model_name_, "None");
  model_state_sub_ = nh.subscribe("/gazebo/model_states", 1, &ModelTFPublisher::modelStatesCallback, this);
  model_names_.clear();
  model_poses_.clear();
  ROS_INFO("Finish Create Instance!!");
}

void ModelTFPublisher::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  model_names_ = msg->name;
  model_poses_ = msg->pose;
}

int ModelTFPublisher::getModelId()
{
  auto iter = std::find(model_names_.begin(), model_names_.end(), urdf_model_name_);
  int index = std::distance(model_names_.begin(), iter);
  if (iter == model_names_.end())
  {
    ROS_WARN_STREAM("Not Found " << urdf_model_name_ << " !!");
    return -1;
  }
  else
  {
    return index;
  }
}

void ModelTFPublisher::broadcastModelTF()
{
  int index = getModelId();
  if (index < 0)
  {
    ROS_WARN_STREAM(urdf_model_name_ << " is not Exsistence!!");
  }
  else
  {
    tf_stamp_.header.stamp = ros::Time::now();
    tf_stamp_.header.frame_id = src_frame_;
    tf_stamp_.child_frame_id = model_names_[index];
    tf_stamp_.transform.translation.x = model_poses_[index].position.x;
    tf_stamp_.transform.translation.y = model_poses_[index].position.y;
    tf_stamp_.transform.translation.z = model_poses_[index].position.z;
    tf_stamp_.transform.rotation.x = model_poses_[index].orientation.x;
    tf_stamp_.transform.rotation.y = model_poses_[index].orientation.y;
    tf_stamp_.transform.rotation.z = model_poses_[index].orientation.z;
    tf_stamp_.transform.rotation.w = model_poses_[index].orientation.w;

    br_.sendTransform(tf_stamp_);
    ROS_INFO_STREAM("Broadcast " << model_names_[index] << " TF !!");
  }
}
