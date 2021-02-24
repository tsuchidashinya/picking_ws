#include <denso_gazebo_control/gazebo_grasp_contacts_bridge.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/common/common.hh>

using gazebo_grasp_contacts_bridge::GazeboGraspContactsBridge;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_grasp_contacts_bridge_node");
  gazebo::client::setup(argc, argv);

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  GazeboGraspContactsBridge ggc_bridge(nh);

  while (ros::ok())
  {
    gazebo::common::Time::MSleep(20);
    ros::spinOnce();
  }

  gazebo::client::shutdown();
}