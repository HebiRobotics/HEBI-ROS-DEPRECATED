#include "hebiros.hpp"


//Advertise topics and services for a group
//Setup a feedback handler to receive feedback from a group
void Hebiros_Node::register_group(std::string group_name) {

  publishers["/hebiros/"+group_name+"/feedback"] =
    n.advertise<FeedbackMsg>("/hebiros/"+group_name+"/feedback", 100);

  publishers["/hebiros/"+group_name+"/feedback/joint_state"] =
    n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/feedback/joint_state", 100);

  publishers["/hebiros/"+group_name+"/command/joint_state"] =   
    n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100);

  subscribers["/hebiros/"+group_name+"/command"] = 
    n.subscribe<CommandMsg>("/hebiros/"+group_name+"/command", 100,
    boost::bind(&Hebiros_Node::sub_command, this, _1, group_name));

  subscribers["/hebiros/"+group_name+"/command/joint_state"] = 
    n.subscribe<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100,
    boost::bind(&Hebiros_Node::sub_joint_command, this, _1, group_name));

  services["/hebiros/"+group_name+"/size"] =
    n.advertiseService<SizeSrv::Request, SizeSrv::Response>("/hebiros/"+group_name+"/size",
    boost::bind(&Hebiros_Node::srv_size, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/set_feedback_frequency"] =
    n.advertiseService<SetFeedbackFrequencySrv::Request,
    SetFeedbackFrequencySrv::Response>(
    "/hebiros/"+group_name+"/set_feedback_frequency",
    boost::bind(&Hebiros_Node::srv_set_feedback_frequency, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/set_command_lifetime"] =
    n.advertiseService<SetCommandLifetimeSrv::Request,
    SetCommandLifetimeSrv::Response>(
    "/hebiros/"+group_name+"/set_command_lifetime",
    boost::bind(&Hebiros_Node::srv_set_command_lifetime, this, _1, _2, group_name));

  services["/hebiros/"+group_name+"/send_command_with_acknowledgement"] =
    n.advertiseService<SendCommandWithAcknowledgementSrv::Request,
    SendCommandWithAcknowledgementSrv::Response>(
    "/hebiros/"+group_name+"/send_command_with_acknowledgement",
    boost::bind(&Hebiros_Node::srv_send_command_with_acknowledgement, this, _1, _2, group_name));

  trajectory_actions[group_name] = std::make_shared<
    actionlib::SimpleActionServer<TrajectoryAction>>(
    n, "/hebiros/"+group_name+"/trajectory",
    boost::bind(&Hebiros_Node::action_trajectory, this, _1, group_name), false);

  trajectory_actions[group_name]->start();

  if (!use_gazebo) {
    std::shared_ptr<Group> group = groups[group_name];
    group_infos[group_name] = new GroupInfo(group->size());
    group->requestInfo(group_infos[group_name]);

    group->addFeedbackHandler([this, group_name](const GroupFeedback& group_fbk) {
      this->publish_group(group_name, group_fbk);
    });

    group->setFeedbackFrequencyHz(feedback_frequency);
    group->setCommandLifetimeMs(command_lifetime);
  }
  else {
    publishers["/hebiros_gazebo_plugin/command"] =
      n.advertise<CommandMsg>("hebiros_gazebo_plugin/command", 100);

    clients["/hebiros_gazebo_plugin/set_command_lifetime"] =
      n.serviceClient<SetCommandLifetimeSrv>("/hebiros_gazebo_plugin/set_command_lifetime");
  }
}

//Feedback handler which publishes feedback topics for a group
void Hebiros_Node::publish_group(std::string group_name, const GroupFeedback& group_fbk) {

  FeedbackMsg feedback_msg;
  sensor_msgs::JointState joint_state_msg;

  for (int i = 0; i < group_fbk.size(); i++) {
    std::string name = (*group_infos[group_name])[i].settings().name().get();
    std::string family = (*group_infos[group_name])[i].settings().family().get();
    double position = group_fbk[i].actuator().position().get();
    double velocity = group_fbk[i].actuator().velocity().get();
    double effort = group_fbk[i].actuator().effort().get();

    joint_state_msg.name.push_back(family+"/"+name);
    joint_state_msg.position.push_back(position);
    joint_state_msg.velocity.push_back(velocity);
    joint_state_msg.effort.push_back(effort);

    feedback_msg.name.push_back(family+"/"+name);
    feedback_msg.position.push_back(position);
    feedback_msg.velocity.push_back(velocity);
    feedback_msg.effort.push_back(effort);
    feedback_msg.position_command.push_back(group_fbk[i].actuator().positionCommand().get());
    feedback_msg.velocity_command.push_back(group_fbk[i].actuator().velocityCommand().get());
    feedback_msg.effort_command.push_back(group_fbk[i].actuator().effortCommand().get());
    geometry_msgs::Vector3 accelerometer_msg;
    accelerometer_msg.x = group_fbk[i].imu().accelerometer().get().getX();
    accelerometer_msg.y = group_fbk[i].imu().accelerometer().get().getY();
    accelerometer_msg.z = group_fbk[i].imu().accelerometer().get().getZ();
    feedback_msg.accelerometer.push_back(accelerometer_msg);
    geometry_msgs::Vector3 gyro_msg;
    gyro_msg.x = group_fbk[i].imu().gyro().get().getX();
    gyro_msg.y = group_fbk[i].imu().gyro().get().getY();
    gyro_msg.z = group_fbk[i].imu().gyro().get().getZ();
    feedback_msg.gyro.push_back(gyro_msg);
    feedback_msg.deflection.push_back(group_fbk[i].actuator().deflection().get());
    feedback_msg.deflection_velocity.push_back(group_fbk[i].actuator().deflectionVelocity().get());
    feedback_msg.motor_velocity.push_back(group_fbk[i].actuator().motorVelocity().get());
    feedback_msg.motor_current.push_back(group_fbk[i].actuator().motorCurrent().get());
    feedback_msg.motor_winding_current.push_back(group_fbk[i].actuator().motorWindingCurrent().get());
    feedback_msg.motor_sensor_temperature.push_back(
      group_fbk[i].actuator().motorSensorTemperature().get());
    feedback_msg.motor_winding_temperature.push_back(
      group_fbk[i].actuator().motorWindingTemperature().get());
    feedback_msg.motor_housing_temperature.push_back(
      group_fbk[i].actuator().motorHousingTemperature().get());
    feedback_msg.board_temperature.push_back(group_fbk[i].boardTemperature().get());
    feedback_msg.processor_temperature.push_back(group_fbk[i].processorTemperature().get());
    feedback_msg.voltage.push_back(group_fbk[i].voltage().get());
    std_msgs::ColorRGBA led_msg;
    led_msg.r = group_fbk[i].led().getColor().getRed();
    led_msg.g = group_fbk[i].led().getColor().getGreen();
    led_msg.b = group_fbk[i].led().getColor().getBlue();
    if (group_fbk[i].led().hasColor()) {
      led_msg.a = 255;
    }
    else {
      led_msg.a = 0;
    }
    feedback_msg.led_color.push_back(led_msg);
    feedback_msg.sequence_number.push_back(group_fbk[i].actuator().sequenceNumber().get());
    feedback_msg.receive_time.push_back(group_fbk[i].actuator().receiveTime().get());
    feedback_msg.transmit_time.push_back(group_fbk[i].actuator().transmitTime().get());
    feedback_msg.hardware_receive_time.push_back(group_fbk[i].actuator().hardwareReceiveTime().get());
    feedback_msg.hardware_transmit_time.push_back(
      group_fbk[i].actuator().hardwareTransmitTime().get());
  }

  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
  group_joint_states[group_name] = joint_state_msg;
} 

//Deconstruction of a group
void Hebiros_Node::unregister_group(std::string group_name) {
  if (groups[group_name]) {
    std::shared_ptr<Group> group = groups[group_name];
    group->clearFeedbackHandlers();
  }
}


