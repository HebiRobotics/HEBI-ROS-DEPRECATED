#include <hebiros_gazebo_joint.h>

/////////////////////////////////
// TODO: refactor this to be a "Joint Info" structure, owned by the joint, and set
// on construction...
/////////////////////////////////

static constexpr double GEAR_RATIO_X5_1 = 272.22;
static constexpr double GEAR_RATIO_X8_3 = 272.22;
static constexpr double GEAR_RATIO_X5_4 = 762.22;
static constexpr double GEAR_RATIO_X8_9 = 762.22;
static constexpr double GEAR_RATIO_X5_9 = 1742.22;
static constexpr double GEAR_RATIO_X8_16 = 1462.222;

static constexpr double DEFAULT_GEAR_RATIO = 272.22;

static std::map<std::string, double> gear_ratios = {
  {"X5_1", GEAR_RATIO_X5_1},
  {"X5_4", GEAR_RATIO_X5_4},
  {"X5_9", GEAR_RATIO_X5_9},
  {"X8_3", GEAR_RATIO_X8_3},
  {"X8_9", GEAR_RATIO_X8_9},
  {"X8_16", GEAR_RATIO_X8_16}
};

double getGearRatio(const std::string& model_name) {
  if (gear_ratios.find(model_name) != gear_ratios.end()) {
    return gear_ratios[model_name];
  }
  return DEFAULT_GEAR_RATIO;
}

double getVelocityFF(double gear_ratio, bool is_x8) {
  double voltage = 48;
  double speed_constant = is_x8 ? 1360 : 1530;
  return gear_ratio / (speed_constant * (2.0 * M_PI / 60.0) * voltage);
}

double getEffortFF(double gear_ratio, bool is_x8) {
  double voltage = 48;
  double term_resistance = is_x8 ? 1360 : 1530;
  double torque_constant = is_x8 ? (9.32f / 1000.0f) : (6.26f / 1000.0f);
  return term_resistance / (gear_ratio * voltage * torque_constant * 0.65);
}

/////////////////////////////////
// END REFACTOR TODO
/////////////////////////////////

HebirosGazeboJoint::HebirosGazeboJoint(const std::string& name_,
  const std::string& model_name_,
  bool is_x8, // TODO: do this better...
  std::shared_ptr<ros::NodeHandle> n)
  : name(name_), model_name(model_name_),
    temperature(is_x8 ?
      hebi::sim::TemperatureModel::createX8() :
      hebi::sim::TemperatureModel::createX5()),
    gear_ratio(getGearRatio(model_name)),
    position_pid(1),
    velocity_pid(getVelocityFF(gear_ratio, is_x8)),
    effort_pid(getEffortFF(gear_ratio, is_x8)) {
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
