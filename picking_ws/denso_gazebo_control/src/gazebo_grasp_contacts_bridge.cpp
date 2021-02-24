#include <denso_gazebo_control/gazebo_grasp_contacts_bridge.h>

#include <vector>

#include <denso_gazebo_msgs/Contact.h>
#include <denso_gazebo_msgs/ContactArray.h>

using gazebo_grasp_contacts_bridge::GazeboGraspContactsBridge;

static const double AIRBORNE_THRESHOLD = 0.3;

GazeboGraspContactsBridge::GazeboGraspContactsBridge(ros::NodeHandle& nh)
  : nh_(nh), airborne_(false), node_(new gazebo::transport::Node())
{
  ros::param::param<std::string>("~robot_name", robot_name_, "vs087");
  node_->Init();
  contacts_pub_ = nh.advertise<denso_gazebo_msgs::ContactArray>("/" + robot_name_ + "/grasp_contacts", 1);
  position_sub_ = nh.subscribe("ground_truth/state", 1000, &GazeboGraspContactsBridge::positionCb, this);
  contacts_sub_ =
      node_->Subscribe("/gazebo/default/" + robot_name_ + "/contacts", &GazeboGraspContactsBridge::forcesCb, this);
  ROS_INFO("Initialize GazeboGraspContactsBridge instance !!");
}

GazeboGraspContactsBridge::~GazeboGraspContactsBridge()
{
}

// Forces callback function
void GazeboGraspContactsBridge::forcesCb(ConstContactsPtr& msg)
{
  denso_gazebo_msgs::ContactArray contacts_message;
  std::vector<denso_gazebo_msgs::Contact> contacts_list;
  // What to do when callback
  for (int i = 0; i < msg->contact_size(); ++i)
  {
    denso_gazebo_msgs::Contact contact_message;

    contact_message.collision_1 = msg->contact(i).collision1();
    contact_message.collision_2 = msg->contact(i).collision2();

    contact_message.normal[0] = msg->contact(i).normal().Get(0).x();
    contact_message.normal[1] = msg->contact(i).normal().Get(0).y();
    contact_message.normal[2] = msg->contact(i).normal().Get(0).z();

    contact_message.position[0] = msg->contact(i).position().Get(0).x();
    contact_message.position[1] = msg->contact(i).position().Get(0).y();
    contact_message.position[2] = msg->contact(i).position().Get(0).z();

    contact_message.force[0] = msg->contact(i).wrench().Get(0).body_2_wrench().force().x();
    contact_message.force[1] = msg->contact(i).wrench().Get(0).body_2_wrench().force().y();
    contact_message.force[2] = msg->contact(i).wrench().Get(0).body_2_wrench().force().z();

    contact_message.torque[0] = msg->contact(i).wrench().Get(0).body_2_wrench().torque().x();
    contact_message.torque[1] = msg->contact(i).wrench().Get(0).body_2_wrench().torque().y();
    contact_message.torque[2] = msg->contact(i).wrench().Get(0).body_2_wrench().torque().z();

    contact_message.depth = msg->contact(i).depth().Get(0);

    contacts_list.push_back(contact_message);
  }
  if (msg->contact_size() == 0)
  {
    denso_gazebo_msgs::Contact contact_message;

    contact_message.collision_1 = "None";
    contact_message.collision_2 = "None";

    contact_message.normal[0] = 0;
    contact_message.normal[1] = 0;
    contact_message.normal[2] = 0;

    contact_message.position[0] = 0;
    contact_message.position[1] = 0;
    contact_message.position[2] = 0;

    contact_message.force[0] = 0;
    contact_message.force[1] = 0;
    contact_message.force[2] = 0;

    contact_message.torque[0] = 0;
    contact_message.torque[1] = 0;
    contact_message.torque[2] = 0;

    contact_message.depth = 0;

    contacts_list.push_back(contact_message);
  }
  contacts_message.contacts = contacts_list;
  contacts_pub_.publish(contacts_message);
}

// Position callback function
void GazeboGraspContactsBridge::positionCb(const nav_msgs::Odometry::ConstPtr& msg2)
{
  if (msg2->pose.pose.position.z > AIRBORNE_THRESHOLD)
  {
    airborne_ = true;
  }
  else
  {
    airborne_ = false;
  }
}
