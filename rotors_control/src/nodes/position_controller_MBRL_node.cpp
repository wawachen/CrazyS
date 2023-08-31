/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2020 Ria Sonecha, MIT, USA
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

#include "position_controller_MBRL_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"
#include "rotors_control/position_controller_request.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

PositionControllerMBRLNode::PositionControllerMBRLNode() {

    ROS_INFO_ONCE("Started position controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

    position_controller_cal_ = nh.advertiseService("position_controller_control",&PositionControllerMBRLNode::pos_service_callback,this);

}

PositionControllerMBRLNode::~PositionControllerMBRLNode(){}

bool PositionControllerMBRLNode::pos_service_callback(rotors_control::position_controller_request_full::Request  &req, rotors_control::position_controller_request_full::Response &res)
{
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(req.x, req.y, req.z);
  double desired_yaw = req.yaw;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(trajectory_msg.points.front(), &eigen_reference);

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);

  EigenOdometry odometry;
  eigenOdometryFromMsg1(req.odometry, &odometry);
  position_controller_.SetOdometryWithoutStateEstimator(odometry);

  Eigen::Vector4d ref_rotor_velocities;
  position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  res.m1 = ref_rotor_velocities[0];
  res.m2 = ref_rotor_velocities[1];
  res.m3 = ref_rotor_velocities[2];
  res.m4 = ref_rotor_velocities[3];

  return true;

}


void PositionControllerMBRLNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/x",
                  position_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/y",
                  position_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "xy_gain_ki/x",
                  position_controller_.controller_parameters_.xy_gain_ki_.x(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(pnh, "xy_gain_ki/y",
                  position_controller_.controller_parameters_.xy_gain_ki_.y(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  position_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  position_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  position_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  position_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  position_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  position_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  position_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  position_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "yaw_gain_kp/yaw",
                  position_controller_.controller_parameters_.yaw_gain_kp_,
                  &position_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(pnh, "yaw_gain_ki/yaw",
                  position_controller_.controller_parameters_.yaw_gain_ki_,
                  &position_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(pnh, "hovering_gain_kp/z",
                  position_controller_.controller_parameters_.hovering_gain_kp_,
                  &position_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(pnh, "hovering_gain_ki/z",
                  position_controller_.controller_parameters_.hovering_gain_ki_,
                  &position_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(pnh, "hovering_gain_kd/z",
                  position_controller_.controller_parameters_.hovering_gain_kd_,
                  &position_controller_.controller_parameters_.hovering_gain_kd_);

  position_controller_.SetControllerGains();

  ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");

  //Reading the parameters come from the launch file
  bool dataStoringActive;
  double dataStoringTime;
  std::string user;

  if (pnh.getParam("user_account", user)){
  ROS_INFO("Got param 'user_account': %s", user.c_str());
  position_controller_.user_ = user;
  }
  else
      ROS_ERROR("Failed to get param 'user'");

  if (pnh.getParam("csvFilesStoring", dataStoringActive)){
  ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
  position_controller_.dataStoring_active_ = dataStoringActive;
  }
  else
      ROS_ERROR("Failed to get param 'csvFilesStoring'");

  if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
  ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
  position_controller_.dataStoringTime_ = dataStoringTime;
  }
  else
      ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

  position_controller_.SetLaunchFileParameters();


}

void PositionControllerMBRLNode::Publish(){
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_MBRL_node_with_stateEstimator");

    ros::NodeHandle nh2;

    rotors_control::PositionControllerMBRLNode position_controller_node;

    ros::spin();

    return 0;
}
