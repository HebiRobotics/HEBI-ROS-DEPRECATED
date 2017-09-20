#include "hebiros.hpp"


int main(int argc, char **argv) {

  ros::init(argc, argv, "hebiros_node");
  Hebiros_Node node(argc, argv);

  return 0;
}

