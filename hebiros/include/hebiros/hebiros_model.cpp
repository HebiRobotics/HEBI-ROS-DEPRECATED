#include "hebiros_model.h"

#include "hebiros.h"


HebirosModel::HebirosModel(std::string model_name) {
  this->name = name;

  if (loadURDF()) {
    createModel();
  }
}

bool HebirosModel::loadURDF() {

  std::string urdf_name("robot_description");
  urdf::Model urdf_model;

  if (!urdf_model.initParam(urdf_name))
  {
    ROS_WARN("Could not load robot_description");
  }
  else {
    ROS_INFO("Loaded URDF from robot_description");

    this->urdf_model = urdf_model;
  }
}

void printChild(const urdf::Link* link) {

  for (auto& joint : link->child_joints) {
    std::cout << "joint ";

    if (joint->type == urdf::Joint::FIXED) {
      std::cout << "(fixed): ";
    }
    else {
      std::cout << "(not-fixed): ";
    }

    std::cout << joint->name << std::endl;
  }

  for (auto& link_child : link->child_links) {
    std::cout << "child: " << link_child.get()->name << std::endl;
    printChild(link_child.get());
  }

}

void HebirosModel::createModel() {

    const urdf::Link* link = urdf_model.getRoot().get();

    std::cout << "root: " << link->name << std::endl;
    printChild(link);

}



