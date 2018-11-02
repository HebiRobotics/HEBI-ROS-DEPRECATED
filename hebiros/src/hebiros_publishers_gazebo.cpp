#include "hebiros_publishers_gazebo.h"

#include "hebiros.h"

using namespace hebiros;


void HebirosPublishersGazebo::registerGroupPublishers(std::string group_name) {

  publishers["hebiros_gazebo_plugin/command/"+group_name] =
    HebirosNode::n_ptr->advertise<CommandMsg>(
    "hebiros_gazebo_plugin/command/"+group_name, 100);

  HebirosPublishers::registerGroupPublishers(group_name);
}

void HebirosPublishersGazebo::command(CommandMsg command_msg, std::string group_name) {

  publishers["hebiros_gazebo_plugin/command/"+group_name].publish(command_msg);
}

