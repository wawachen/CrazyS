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

#include "lee_position_controller_node_v2.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

  pause_signal_sub_ = nh.subscribe("/pause_controller", 1, &LeePositionControllerNode::pause_f_Callback,this);

  cmd_pos_trajectory_sub_ = nh.subscribe(
      "cmd_pos", 1,
      &LeePositionControllerNode::posTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LeePositionControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  _pause_s = 0;

}

LeePositionControllerNode::~LeePositionControllerNode() { }

void LeePositionControllerNode::InitializeParams() {
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
void LeePositionControllerNode::Publish() {
}

void LeePositionControllerNode::pause_f_Callback(const std_msgs::Int16ConstPtr& msg)
{
  _pause_s = msg->data;
  // if (_pause_s == 1){
  //   reset_controller();}
}

// void LeePositionControllerNode::reset_controller(){
//   lee_position_controller_.Reset_controller();
//   ros::NodeHandle pnh("~");

//   // Read parameters from rosparam.
//   GetRosParameter(pnh, "position_gain/x",
//                   lee_position_controller_.controller_parameters_.position_gain_.x(),
//                   &lee_position_controller_.controller_parameters_.position_gain_.x());
//   GetRosParameter(pnh, "position_gain/y",
//                   lee_position_controller_.controller_parameters_.position_gain_.y(),
//                   &lee_position_controller_.controller_parameters_.position_gain_.y());
//   GetRosParameter(pnh, "position_gain/z",
//                   lee_position_controller_.controller_parameters_.position_gain_.z(),
//                   &lee_position_controller_.controller_parameters_.position_gain_.z());
//   GetRosParameter(pnh, "velocity_gain/x",
//                   lee_position_controller_.controller_parameters_.velocity_gain_.x(),
//                   &lee_position_controller_.controller_parameters_.velocity_gain_.x());
//   GetRosParameter(pnh, "velocity_gain/y",
//                   lee_position_controller_.controller_parameters_.velocity_gain_.y(),
//                   &lee_position_controller_.controller_parameters_.velocity_gain_.y());
//   GetRosParameter(pnh, "velocity_gain/z",
//                   lee_position_controller_.controller_parameters_.velocity_gain_.z(),
//                   &lee_position_controller_.controller_parameters_.velocity_gain_.z());
//   GetRosParameter(pnh, "attitude_gain/x",
//                   lee_position_controller_.controller_parameters_.attitude_gain_.x(),
//                   &lee_position_controller_.controller_parameters_.attitude_gain_.x());
//   GetRosParameter(pnh, "attitude_gain/y",
//                   lee_position_controller_.controller_parameters_.attitude_gain_.y(),
//                   &lee_position_controller_.controller_parameters_.attitude_gain_.y());
//   GetRosParameter(pnh, "attitude_gain/z",
//                   lee_position_controller_.controller_parameters_.attitude_gain_.z(),
//                   &lee_position_controller_.controller_parameters_.attitude_gain_.z());
//   GetRosParameter(pnh, "angular_rate_gain/x",
//                   lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
//                   &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
//   GetRosParameter(pnh, "angular_rate_gain/y",
//                   lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
//                   &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
//   GetRosParameter(pnh, "angular_rate_gain/z",
//                   lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
//                   &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
//   GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
//   lee_position_controller_.InitializeParameters();

// }

void LeePositionControllerNode::posTrajectoryCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{   
  float x, y, z, yaw;
  x = msg->pose.position.x;
  y = msg->pose.position.y;
  z = msg->pose.position.z;
  yaw = msg->pose.orientation.w;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  const float DEG_2_RAD = M_PI / 180.0;

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(x, y, z);
  double desired_yaw = yaw*DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(trajectory_msg.points.front(), &eigen_reference);

  // We can trigger the first command immediately.
  // if (_pause_s == 0)
  // {lee_position_controller_.SetTrajectoryPoint(eigen_reference);}
  lee_position_controller_.SetTrajectoryPoint(eigen_reference);
}

void LeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  Eigen::VectorXd ref_rotor_velocities;

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
  
  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  if (_pause_s == 0){
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  }
  else{
    for (int i = 0; i < 6; i++)
      actuator_msg->angular_velocities.push_back(0.0);
  }
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  rotors_control::LeePositionControllerNode lee_position_controller_node;

  ros::spin();

  return 0;
}
