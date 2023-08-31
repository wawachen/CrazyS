
#ifndef _BOX_PLUGIN_HH_
#define _BOX_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <fstream>
#include <math.h>

#include "ConnectGazeboToRosTopic.pb.h"


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class BoxPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: BoxPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      
      // Store the model pointer for convenience.
      this->model = _model;

      // #if GAZEBO_MAJOR_VERSION < 8
      // this->node->Init(this->model->GetWorld()->GetName());
      // #else
      // this->node->Init(this->model->GetWorld()->Name());
      // #endif
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "box_node",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node.
      this->rosNode.reset(new ros::NodeHandle("box_node_Handle"));

      // Subscribe to the topic, and register a callback
      sub = this->rosNode->subscribe<nav_msgs::Odometry>("/bar/ground_truth/odometry", 1, &BoxPlugin::OnMsg,this);
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BoxPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // ignition::math::Pose3d pose(pos_x, pos_y, 3.0, 0.0, 0.0, 0.0, 0.0);  // = orig_pose;
      ignition::math::Pose3d pose(pos_x, pos_y, 3.0, 0.0, 0.0, 0.0, 0.0);  // = orig_pose;
      this->model->SetWorldPose(pose);
    }

    void OnMsg(const nav_msgs::Odometry::ConstPtr& _msg) 
    {
      pos_x = _msg->pose.pose.position.x;
      pos_y = _msg->pose.pose.position.y;
    }

    /// \brief A node used for transport
    // private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    // private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: double pos_x,pos_y;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    // Create the node
    private: ros::Subscriber sub;


  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(BoxPlugin)
}
#endif

