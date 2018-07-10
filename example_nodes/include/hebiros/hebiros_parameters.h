#ifndef HEBIROS_PARAMETERS_H
#define HEBIROS_PARAMETERS_H

#include "ros/ros.h"


class HebirosParameters {

  public:

    static void setNodeParameters();
    static void loadBool(std::string name);
    static void setBool(std::string name, bool value);
    static bool getBool(std::string name);
    static void loadInt(std::string name);
    static void setInt(std::string name, int value);
    static int getInt(std::string name);

  private:

    static std::map<std::string, bool> bool_parameters_default;
    static std::map<std::string, bool> bool_parameters;
    static std::map<std::string, int> int_parameters_default;
    static std::map<std::string, int> int_parameters;

};

#endif
