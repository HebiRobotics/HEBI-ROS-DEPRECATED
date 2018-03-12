
#include <hebiros_gazebo_joint.h>


HebirosGazeboJoint::HebirosGazeboJoint(std::string name,
  std::shared_ptr<ros::NodeHandle> n) {

  this->name = name;

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
  this->prev_feedback_time = current_time;

  this->imu_subscriber = n->subscribe<sensor_msgs::Imu>(
    "/hebiros_gazebo_plugin/imu/"+name,
    100, boost::bind(&HebirosGazeboJoint::SubIMU, this, _1));
}

HebirosGazeboJoint::~HebirosGazeboJoint() {}

//Subscriber which receives IMU feedback from gazebo
void HebirosGazeboJoint::SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data) {

  this->feedback.accelerometer = {data->linear_acceleration};
  this->feedback.gyro = {data->angular_velocity};
}

//Reset the joint for when a new command is received
void HebirosGazeboJoint::Reset(int i, CommandMsg command_msg) {

  this->command_index = i;
  this->command_target = command_msg;

  ros::Time current_time = ros::Time::now();
  this->start_time = current_time;
  this->prev_time = current_time;
  this->command_received = true;
}

