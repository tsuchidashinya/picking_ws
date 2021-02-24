#include <denso_gazebo/model_tf_publisher.h>

using model_tf_publisher::ModelTFPublisher;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_tf_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ModelTFPublisher br(nh);

  ROS_INFO("Publish Model TF");

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    br.broadcastModelTF();
    loop_rate.sleep();
  }

  return 0;
}
