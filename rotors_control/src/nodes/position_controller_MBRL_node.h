/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2020 Ria Sonecha, MIT, USA
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#ifndef CRAYZFLIE_2_POSITION_CONTROLLER_MBRL_NODE_H
#define CRAYZFLIE_2_POSITION_CONTROLLER_MBRL_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>
#include "rotors_control/position_controller_request_full.h"


#include "rotors_control/common.h"
#include "rotors_control/position_controller.h"
#include "rotors_control/mellinger_controller.h"
#include "rotors_control/internal_model_controller.h"
#include "rotors_control/crazyflie_complementary_filter.h"


namespace rotors_control {

    class PositionControllerMBRLNode{
        public:
            PositionControllerMBRLNode();
            ~PositionControllerMBRLNode();

            void InitializeParams();
            void Publish();

        private:
            // bool waypointHasBeenPublished_ = false;

            PositionController position_controller_;
            sensorData_t sensors_;
            ros::Time imu_msg_head_stamp_;

            std::string namespace_;

            ros::NodeHandle n_;

            ros::ServiceServer position_controller_cal_;

            bool pos_service_callback(rotors_control::position_controller_request_full::Request  &req, rotors_control::position_controller_request_full::Response &res);

    };
}

#endif // CRAZYFLIE_2_POSITION_CONTROLLER_MBRL_NODE_H
