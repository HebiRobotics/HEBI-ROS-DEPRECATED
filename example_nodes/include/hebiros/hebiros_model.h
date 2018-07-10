#ifndef HEBIROS_MODEL_H
#define HEBIROS_MODEL_H

#include "ros/ros.h"
#include "urdf/model.h"
#include "robot_model.hpp"

class HebirosModel {

  public:
    // Tries to read the model from the robot description parameter. Add to the
    // set of models and returns true on success, otherwise returns false.
    static bool load(const std::string& name, const std::string& description_param);

    // Technically, this is only used by this class, but it is used by the 
    // std::map contained herein, so it has to be public
    HebirosModel(std::unique_ptr<hebi::robot_model::RobotModel> model_);

    static HebirosModel* getModel(const std::string& model_name);

    hebi::robot_model::RobotModel& getModel();

  private:
    // Name to imported model map; filled in by "load"
    static std::map<std::string, HebirosModel> models;

    // The underlying C++ API model that is "owned" by this HebirosModel
    std::unique_ptr<hebi::robot_model::RobotModel> model;
   
    // Loads any model on the given parameter name into the URDF model 
    static bool loadURDF(const std::string& description_param, urdf::Model& model);

    // Create a hebi robot model from the URDF; return object on success, empty
    // pointer on failure.
    static std::unique_ptr<hebi::robot_model::RobotModel> parseURDF(const urdf::Model& model);
};

#endif
