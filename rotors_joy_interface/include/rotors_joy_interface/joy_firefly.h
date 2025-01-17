/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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


#ifndef ROTORS_JOY_INTERFACE_JOY_FIREFLY_H_
#define ROTORS_JOY_INTERFACE_JOY_FIREFLY_H_

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

struct Axes {
  int vx;
  int vy;
  int vz;
};

struct Buttons {
  int shutdown;
};


class Joy {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub_;
  ros::Publisher shut_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;

  Axes axes_;
  Buttons buttons_;

  geometry_msgs::Vector3 vel_;
  sensor_msgs::Joy current_joy_;
  std_msgs::Int16 signal_;

  double current_pos_x_;
  double current_pos_y_;
  double current_pos_z_;

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

 public:
  Joy();
};

#endif // ROTORS_JOY_INTERFACE_JOY_FIREFLY_H_
