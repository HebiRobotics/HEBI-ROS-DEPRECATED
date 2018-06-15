
#ifndef _HEBIROS_GAZEBO_JOINT_HH_
#define _HEBIROS_GAZEBO_JOINT_HH_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

// TODO: split into model/state? Or something?
// A simple multi-stage temperature model.
class TemperatureModel {

  public:
		
		// Perform an update of all of the stages of the model,
		// based on the estimated power going into the system
		void update(double power_in, double dt) {
			// The heat flux between each stage; we calculate
			// these all _before_ updating the temperatures
			double q_wh = (t_w - t_h) / r_wh;
			double q_hb = (t_h - t_b) / r_hb;
			double q_be = (t_b - t_e) / r_be;
			
			// Update the temperature of each phase
			t_w += (power_in - q_wh) * dt / c_w;
			t_h += (q_wh - q_hb) * dt / c_h;
			t_b += (q_hb - q_be) * dt / c_b;
		}

		// Winding temperature
		double getMotorWindingTemperature() { return t_w; }
		double getMotorHousingTemperature() { return t_h; }
		double getActuatorBodyTemperature() { return t_b; }

	private:
		// State variables - these are the bodies for which we
		// model temperature
		double t_w{34};//{130}; // Motor winding
		double t_h{34};//{100}; // Motor housing
		double t_b{34};//{23}; // Actuator body
		static constexpr double t_e{34}; // Environment temperature (in *C) ; assume room temperature + some due to actuator fudge factor
		
	
		// Thermal capacitance and resistance of each modeled body

    // X5 values
		static constexpr double r_wh{2.57f * 4.5f}; // datasheet
		static constexpr double r_hb{1.0f};//{23.5f / 23.5f}; // Open air from data sheet divided my empirical factor
		static constexpr double r_be{23.5f}; // body to environment (empirical; TODO: TUNE)
		static constexpr double c_w{(0.943f) / (2.57)}; // datasheet
		static constexpr double c_h{390.0f / 23.5f}; // housing; datasheet (TC / R)
		static constexpr double c_b{10000.0f / 23.5f}; // actuator body; empirical TODO: TUNE

    // TODO: support X8, too!
    // X8 values
/*		constexpr double r_wh{1.41 * 1.6};
		constexpr double r_hb{17.7f / 17.7f}; // Open air from data sheet divided my empirical factor
		constexpr double r_be{0.25}; // body to environment (empirical; TODO: TUNE)
		constexpr double c_w{(0.9 * 1.6) / (1.41 * 1.6)}; // tc_winding / rth_winding
		constexpr double c_h{1.3}; // housing; empirical TODO: TUNE
		constexpr double c_b{40}; // actuator body; empirical TODO: TUNE
*/
};

class HebirosGazeboJoint : public std::enable_shared_from_this<HebirosGazeboJoint> {

  public:

    std::string name;
    std::string model_name;
    geometry_msgs::Vector3 accelerometer;
    geometry_msgs::Vector3 gyro;

    int feedback_index;
    int command_index;

		TemperatureModel temp {};

    double prev_force {};
    double low_pass_alpha {};
    double gear_ratio {};
    double position_prev_error {};
    double position_elapsed_error {};
    double velocity_prev_error {};
    double velocity_elapsed_error {};
    double effort_prev_error {};
    double effort_elapsed_error {};

    ros::Subscriber imu_subscriber;

    HebirosGazeboJoint(std::string name, std::shared_ptr<ros::NodeHandle> n);
    ~HebirosGazeboJoint();
    void SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data);

};


#endif
