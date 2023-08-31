/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_MBRL_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerMBRLNode::LeePositionControllerMBRLNode() {
  InitializeParams();

  ros::NodeHandle nh;

  // cmd_pose_sub_ = nh.subscribe(
  //     mav_msgs::default_topics::COMMAND_POSE, 1,
  //     &LeePositionControllerNode::CommandPoseCallback, this);

  // cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
  //     mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
  //     &LeePositionControllerNode::MultiDofJointTrajectoryCallback, this);

  // odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
  //                              &LeePositionControllerNode::OdometryCallback, this);

  // motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
  //     mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  // command_timer_ = nh.createTimer(ros::Duration(0), &LeePositionControllerNode::TimedCommandCallback, this,
  //                                 true, false);
  position_controller_cal_ = nh.advertiseService("position_controller_control",&LeePositionControllerMBRLNode::pos_service_callback,this);
  trajectory_set_ = nh.advertiseService("trajectory_setting",&LeePositionControllerMBRLNode::trajectory_callback,this);

}

LeePositionControllerMBRLNode::~LeePositionControllerMBRLNode() { }

void LeePositionControllerMBRLNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}
void LeePositionControllerMBRLNode::Publish() {
}

bool LeePositionControllerMBRLNode::trajectory_callback(rotors_control::trajectory_set::Request  &req, rotors_control::trajectory_set::Response &res)
{
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  const float DEG_2_RAD = M_PI / 180.0;

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(req.x, req.y, req.z);
  double desired_yaw = req.yaw*DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(trajectory_msg.points.front(), &eigen_reference);

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(eigen_reference);

  return true;
  
}

bool LeePositionControllerMBRLNode::pos_service_callback(rotors_control::position_controller_request::Request  &req, rotors_control::position_controller_request::Response &res)
{
  EigenOdometry odometry;
  eigenOdometryFromMsg1(req.odometry, &odometry);

  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  res.m1 = ref_rotor_velocities[0];
  res.m2 = ref_rotor_velocities[1];
  res.m3 = ref_rotor_velocities[2];
  res.m4 = ref_rotor_velocities[3];
  res.m5 = ref_rotor_velocities[4];
  res.m6 = ref_rotor_velocities[5];

  return true;
  
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_MBRL_node");

  rotors_control::LeePositionControllerMBRLNode lee_position_controller_MBRL_node;

  ros::spin();

  return 0;
}
