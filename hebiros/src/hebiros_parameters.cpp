#include "hebiros_parameters.h"

#include "hebiros.h"


std::map<std::string, bool> HebirosParameters::bool_parameters_default =
  {{"use_sim_time", false}};
std::map<std::string, bool> HebirosParameters::bool_parameters;
std::map<std::string, int> HebirosParameters::int_parameters_default =
  {{"hebiros/node_frequency", 200},
   {"hebiros/action_frequency", 200},
   {"hebiros/feedback_frequency", 100},
   {"hebiros/command_lifetime", 100}};
std::map<std::string, int> HebirosParameters::int_parameters;

void HebirosParameters::setNodeParameters() {

  loadInt("hebiros/node_frequency");
  loadInt("hebiros/action_frequency");
  loadInt("hebiros/feedback_frequency");
  loadInt("hebiros/command_lifetime");

  ROS_INFO("Parameters:");
  ROS_INFO("hebiros/node_frequency=%d", getInt("hebiros/node_frequency"));
  ROS_INFO("hebiros/action_frequency=%d", getInt("hebiros/action_frequency"));
  ROS_INFO("hebiros/feedback_frequency=%d", getInt("hebiros/feedback_frequency"));
  ROS_INFO("hebiros/command_lifetime=%d", getInt("hebiros/command_lifetime"));
}

void HebirosParameters::loadBool(std::string name) {
  if (bool_parameters_default.find(name) != bool_parameters_default.end()) {
    bool value;
    bool default_value = bool_parameters_default[name];

    HebirosNode::n_ptr->param<bool>(name, value, default_value);
    HebirosNode::n_ptr->setParam(name, value);
    bool_parameters[name] = value;
  }
}

void HebirosParameters::setBool(std::string name, bool value) {

  if (bool_parameters_default.find(name) != bool_parameters_default.end()) {
    HebirosNode::n_ptr->setParam(name, value);
    bool_parameters[name] = value;
  }
}

bool HebirosParameters::getBool(std::string name) {

  if (bool_parameters.find(name) != bool_parameters.end()) {
    return bool_parameters[name];
  }
  else if (bool_parameters_default.find(name) != bool_parameters_default.end()) {
    return bool_parameters_default[name];
  }
  else {
    return 0;
  }
}

void HebirosParameters::loadInt(std::string name) {
  if (int_parameters_default.find(name) != int_parameters_default.end()) {
    int value;
    int default_value = int_parameters_default[name];

    HebirosNode::n_ptr->param<int>(name, value, default_value);
    HebirosNode::n_ptr->setParam(name, value);
    int_parameters[name] = value;
  }
}

void HebirosParameters::setInt(std::string name, int value) {

  if (int_parameters_default.find(name) != int_parameters_default.end()) {
    HebirosNode::n_ptr->setParam(name, value);
    int_parameters[name] = value;
  }
}

int HebirosParameters::getInt(std::string name) {

  if (int_parameters.find(name) != int_parameters.end()) {
    return int_parameters[name];
  }
  else if (int_parameters_default.find(name) != int_parameters_default.end()) {
    return int_parameters_default[name];
  }
  else {
    return 0;
  }
}
