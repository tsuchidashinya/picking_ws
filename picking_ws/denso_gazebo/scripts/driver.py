#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


class DensoRobotArmSim(object):
    def __init__(self):
        self.robot_name_ = rospy.get_param("~robot_name", default="vs087")
        self.robot_controller_ = "/" + self.robot_name_ + "/arm_controller"
        self.arm_joints_state_ = JointTrajectoryControllerState()
        self.arm_joints_command_ = JointTrajectory()
        self.arm_joints_ = rospy.get_param(self.robot_controller_ + "/joints")
        self.arm_joint_pos_min_ = list()
        self.arm_joint_pos_max_ = list()
        self.arm_sub_ = rospy.Subscriber(
            self.robot_controller_ + "/state",
            JointTrajectoryControllerState,
            self.read_arm_callback_,
            queue_size=1)
        self.arm_pub_ = rospy.Publisher(
            self.robot_controller_ +
            "/command",
            JointTrajectory,
            queue_size=1)
        rospy.loginfo("Initialize DensoRobotArmSim Class")

    def register_joints_(self):
        initial_point = JointTrajectoryPoint()
        self.arm_joints_command_.header.stamp = rospy.Time.now()
        self.arm_joints_command_.header.frame_id = "world"

        for i in range(len(self.arm_joints_)):
            self.arm_joints_command_.joint_names.append(self.arm_joints_[i])
            initial_point.positions.append(0)
            pos_min = rospy.get_param(
                "/" +
                self.robot_name_ +
                "/joints_limit/arm/" +
                self.arm_joints_[i] +
                "/min_position")
            self.arm_joint_pos_min_.append(pos_min)
            pos_max = rospy.get_param(
                "/" +
                self.robot_name_ +
                "/joints_limit/arm/" +
                self.arm_joints_[i] +
                "/max_position")
            self.arm_joint_pos_max_.append(pos_max)

        self.arm_joints_command_.points.append(initial_point)

        for i in range(len(self.arm_joints_command_.points)):
            self.arm_joints_command_.points[i].time_from_start = rospy.Duration.from_sec(
                0.01)

    def read_arm_callback_(self, msg):
        self.arm_joints_state_ = msg

    def set_default_position_(self):
        default_pos = [-0.0001745329, -0.35203291,
                       2.27294228, 0.0003490659, 1.22173, 3.14159]

        for i in range(len(self.arm_joints_command_.joint_names)):
            self.arm_joints_command_.points[0].positions[i] = default_pos[i]

        self.arm_pub_.publish(self.arm_joints_command_)

    def set_straight_position_(self):
        for i in range(len(self.arm_joints_command_.joint_names)):
            self.arm_joints_command_.points[0].positions[i] = 0

        self.arm_pub_.publish(self.arm_joints_command_)

    def get_position_(self):
        return self.arm_joints_state_.joint_names, self.arm_joints_state_.actual.positions

    def set_position_(self, values):
        if (len(values) != len(self.arm_joints_command_.joint_names)):
            rospy.logerr(
                "The number of require positions and joints do not match !!")
            return False

        for i in range(len(self.arm_joints_command_.joint_names)):
            if (values[i] < self.arm_joint_pos_min_[i]):
                self.arm_joints_command_.points[0].positions[i] = self.arm_joint_pos_min_[
                    i]
                rospy.logwarn(
                    self.arm_joints_command_.joint_names[i] +
                    ": The lower limit is limited to " +
                    str(
                        self.arm_joint_pos_min_[i]))
                rospy.logwarn("Set " +
                              self.arm_joints_command_.joint_names[i] +
                              " to " +
                              str(self.arm_joint_pos_min_[i]))
            elif (self.arm_joint_pos_max_[i] < values[i]):
                self.arm_joints_command_.points[0].positions[i] = self.arm_joint_pos_max_[
                    i]
                rospy.logwarn(
                    self.arm_joints_command_.joint_names[i] +
                    ": The upper limit is limited to " +
                    str(
                        self.arm_joint_pos_max_[i]))
                rospy.logwarn("Set " +
                              self.arm_joints_command_.joint_names[i] +
                              " to " +
                              str(self.arm_joint_pos_max_[i]))
            else:
                self.arm_joints_command_.points[0].positions[i] = values[i]

        self.arm_pub_.publish(self.arm_joints_command_)

        return True
