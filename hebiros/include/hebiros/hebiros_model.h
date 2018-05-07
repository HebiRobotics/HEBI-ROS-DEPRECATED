#ifndef HEBIROS_MODEL_H
#define HEBIROS_MODEL_H

#include "ros/ros.h"
#include "urdf/model.h"


class HebirosModel {

  public:

    std::string name;
    urdf::Model urdf_model;

    HebirosModel(std::string name);
    bool loadURDF();
    void createModel();

};

#endif
