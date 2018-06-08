#include "hebiros.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "hebiros_node");
  HebirosNode node(argc, argv);

  return 0;
}

