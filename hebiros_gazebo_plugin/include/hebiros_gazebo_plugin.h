
#ifndef _HEBIROS_GAZEBO_PLUGIN_HH_
#define _HEBIROS_GAZEBO_PLUGIN_HH_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"
#include "hebiros/CommandMsg.h"

#include "hebiros_gazebo_joint.h"

using namespace hebiros;
using namespace gazebo;

class HebirosGazeboPlugin: public ModelPlugin {

  public:
    HebirosGazeboPlugin();
    ~HebirosGazeboPlugin();
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo & _info);

  private:
    double DEFAULT_POSITION_KP = 1.0;
    double DEFAULT_POSITION_KI = 0.0;
    double DEFAULT_POSITION_KD = 0.0;
    double DEFAULT_VELOCITY_KP = 0.05;
    double DEFAULT_VELOCITY_KI = 0.0;
    double DEFAULT_VELOCITY_KD = 0.0;
    double DEFAULT_EFFORT_KP = 0.25;
    double DEFAULT_EFFORT_KI = 0.0;
    double DEFAULT_EFFORT_KD = 0.001;

    double command_lifetime = 0.1;

    physics::ModelPtr model;
    event::ConnectionPtr update_connection;
    std::unique_ptr<ros::NodeHandle> n;
    ros::Subscriber command_sub;
    ros::ServiceServer acknowledge_srv;
    bool check_acknowledgement;
    bool acknowledgement;
    std::map<std::string, std::shared_ptr<HebirosGazeboJoint>> hebiros_joints;

    void SubCommand(const boost::shared_ptr<CommandMsg const> data);
    bool SrvAcknowledge(std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    void AddJoint(std::string joint_name);
    void UpdateJoint(std::string joint_name, physics::JointPtr joint);

};


#endif
