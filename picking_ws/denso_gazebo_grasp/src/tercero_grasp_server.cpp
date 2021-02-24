#include <denso_gazebo_grasp/tercero_grasp.h>

using tercero_grasp::TerceroGrasp;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tercero_grasp_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  TerceroGrasp tg(nh);

  ROS_INFO("Ready to tercero grasp server");

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
