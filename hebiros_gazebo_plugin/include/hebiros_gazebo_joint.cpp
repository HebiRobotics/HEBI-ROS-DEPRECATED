#include <hebiros_gazebo_joint.h>

HebirosGazeboJoint::HebirosGazeboJoint(const std::string& name_,
  const std::string& model_name_,
  bool is_x8, // TODO: do this better...
  std::shared_ptr<ros::NodeHandle> n)
  : name(name_), model_name(model_name_),
    temperature(is_x8 ?
      hebiros::sim::TemperatureModel::createX8() :
      hebiros::sim::TemperatureModel::createX5()) {
  this->imu_subscriber = n->subscribe<sensor_msgs::Imu>(
    "hebiros_gazebo_plugin/imu/"+name,
    100, boost::bind(&HebirosGazeboJoint::SubIMU, this, _1));
}

//Subscriber which receives IMU feedback from gazebo
void HebirosGazeboJoint::SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data) {
  this->accelerometer = data->linear_acceleration;
  this->gyro = data->angular_velocity;
}

bool HebirosGazeboJoint::isX8() const {
  return (model_name == "X8_3" || 
          model_name == "X8_9" ||
          model_name == "X8_16");
}
