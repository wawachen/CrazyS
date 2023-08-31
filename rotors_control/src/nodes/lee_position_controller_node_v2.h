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

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H_v2
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H_v2

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

class LeePositionControllerNode {
 public:
  LeePositionControllerNode();
  ~LeePositionControllerNode();

  void InitializeParams();
  void Publish();

 private:

  LeePositionController lee_position_controller_;

  std::string namespace_;
  std::int16_t _pause_s;

  // subscribers
  ros::Subscriber cmd_pos_trajectory_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber pause_signal_sub_;

  ros::Publisher motor_velocity_reference_pub_;

  void posTrajectoryCallback(
      const geometry_msgs::PoseStampedConstPtr& msg);
  
  // void reset_controller();

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void pause_f_Callback(const std_msgs::Int16ConstPtr& msg);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H_v2
