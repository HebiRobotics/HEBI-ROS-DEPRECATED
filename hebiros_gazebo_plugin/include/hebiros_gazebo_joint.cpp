
#include <hebiros_gazebo_joint.h>


// TODO: support X8, too
HebirosGazeboJoint::HebirosGazeboJoint(std::string name,
  std::shared_ptr<ros::NodeHandle> n)
  : temp(hebiros::sim::TemperatureModel::createX5()) {

  this->name = name;

  this->imu_subscriber = n->subscribe<sensor_msgs::Imu>(
    "/hebiros_gazebo_plugin/imu/"+name,
    100, boost::bind(&HebirosGazeboJoint::SubIMU, this, _1));

}

HebirosGazeboJoint::~HebirosGazeboJoint() {}

//Subscriber which receives IMU feedback from gazebo
void HebirosGazeboJoint::SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data) {

  this->accelerometer = data->linear_acceleration;
  this->gyro = data->angular_velocity;
}

