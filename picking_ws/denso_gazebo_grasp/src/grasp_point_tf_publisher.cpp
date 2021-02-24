#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_point_tf_publisher");
  ros::NodeHandle nh;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_stamp;
  std::string urdf_model_name;
  std::string gripper_type;
  std::string tf_name;
  double pos_x;
  double pos_y;
  double pos_z;
  double ori_x;
  double ori_y;
  double ori_z;
  double ori_w;

  ros::param::param<std::string>("~model_name", urdf_model_name, "None");
  ros::param::param<std::string>("~gripper_type", gripper_type, "None");
  ros::param::param<std::string>("~tf_name", tf_name, "grasp_point");
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/pos_x", pos_x, 0.0);
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/pos_y", pos_y, 0.0);
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/pos_z", pos_z, 0.0);
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/ori_x", ori_x, 0.0);
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/ori_y", ori_y, 0.0);
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/ori_z", ori_z, 0.0);
  ros::param::param<double>("/object/" + urdf_model_name + "/" + gripper_type + "/ori_w", ori_w, 0.0);

  tf_stamp.header.frame_id = urdf_model_name;
  tf_stamp.child_frame_id = tf_name;
  tf_stamp.transform.translation.x = pos_x;
  tf_stamp.transform.translation.y = pos_y;
  tf_stamp.transform.translation.z = pos_z;
  tf_stamp.transform.rotation.x = ori_x;
  tf_stamp.transform.rotation.y = ori_y;
  tf_stamp.transform.rotation.z = ori_z;
  tf_stamp.transform.rotation.w = ori_w;

  while (ros::ok())
  {
    tf_stamp.header.stamp = ros::Time::now();
    br.sendTransform(tf_stamp);
    ROS_INFO_STREAM("Broadcast grasp_point TF !!");
  }

  return 0;
}
