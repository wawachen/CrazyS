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


#include "rotors_joy_interface/joy_firefly.h"
#include <geometry_msgs/Vector3.h>
#include <mav_msgs/default_topics.h>
#include <std_msgs/Int16.h>

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<geometry_msgs::Vector3> (
    "/goal_pos", 10);
  shut_pub_ = nh_.advertise<std_msgs::Int16> (
    "/shutdown_signal", 10);
  
  current_pos_x_ = 0.0 ;
  current_pos_y_ = 0.0;
  current_pos_z_ = 0.0;

  vel_.x = 0.0;
  vel_.y = 0.0;
  vel_.z = 0.0;
  signal_.data = 0;

  pnh.param("axis_vx_", axes_.vx, 3);
  pnh.param("axis_vz_", axes_.vz, 1);
  pnh.param("axis_vy_", axes_.vy, 4);
  pnh.param("button_shutdown_", buttons_.shutdown, 1);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  vel_.x = msg->axes[axes_.vx];
  vel_.y = msg->axes[axes_.vy];
  vel_.z = msg->axes[axes_.vz];

  if (msg->buttons[buttons_.shutdown]) {
    signal_.data = 1;
  }else{
    signal_.data = 0;
  }

  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(vel_);
  shut_pub_.publish(signal_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Joy joy;

  ros::spin();

  return 0;
}
