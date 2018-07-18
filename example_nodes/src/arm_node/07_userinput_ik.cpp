#include <atomic>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "trajectory.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "util/grav_comp.hpp"
#include "ros/ros.h"
#include "tomatobot/imgData.h"
#include "tomatobot/imgDataArray.h"
#include "tomatobot/xyz.h"
#include "tomatobot/xyzArray.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#define ERROR_THRESH 0.1

bool target;
Eigen::Vector3d target_xyz;
Eigen::MatrixXf target_matrix(3,20);
int commandSize;

using namespace hebi;
using ActuatorType = robot_model::RobotModel::ActuatorType;
using BracketType = robot_model::RobotModel::BracketType;
using LinkType = robot_model::RobotModel::LinkType;

/// For printing string vectors
std::ostream& operator	<<(std::ostream& os, const std::vector<std::string>& vec) {
    os << '{';
    size_t size = vec.size();
    for (size_t i = 0; i < size - 1; i++) {
        os << '"' << vec[i] << "\", ";
    }
    if (size > 0) {
        os << '"' << vec[size - 1] << '"';
    }
    return os << '}';
}

static void getInput(Eigen::Vector3d& target_xyz){
	//Adding script to get target
    double x = 0;
	double y = 0;
	double z = 0;

	std::cout << "Please enter x: ";
	std::cin >> x;
	std::cout << "Please enter y: ";
	std::cin >> y;
	std::cout << "Please enter z: ";
	std::cin >> z;
	target_xyz << x, y, z;	
}

namespace hebi {
namespace example {

/**
 * Structure of initialization parameters.
 */
struct CreateDesc {
    std::vector<std::string> moduleNames;
    std::vector<std::string> familyNames;
    std::vector<double> homePosition;       /// Home Position of the arm
    int commandLifetime;                    /// Lifetime of command (in milliseconds)
    double waypointTransitionTime;          /// Time (in seconds) to move from one waypoint to the next
    double feedbackFrequency;               /// Feedback frequency (in Hertz)
};

///**
// * Fetches the group from the provided description
// */
static bool createGroup(CreateDesc& createDesc, std::shared_ptr<Group>& group);

static bool createGroup(CreateDesc& createDesc, std::shared_ptr<Group>& group) {
    Lookup lookup;
    group = lookup.getGroupFromNames(createDesc.familyNames, createDesc.moduleNames);

    if (group) {
        group->setCommandLifetimeMs(createDesc.commandLifetime);
        group->setFeedbackFrequencyHz(createDesc.feedbackFrequency);
        return true;
    }

    std::cerr << "Lookup::getGroupFromNames("
        << createDesc.moduleNames << ", "
        << createDesc.familyNames << ")" << " returned null" << std::endl;
    return false;
}


static void moveToWayPoint(Group* group,
const robot_model::RobotModel& model, const Eigen::VectorXd& masses, const Eigen::Vector3d& gravity, Eigen::VectorXd* endPoint, double timeDuration) {
    const int numberOfModules = group->size();
    GroupFeedback feedback(numberOfModules);
    GroupCommand cmd(numberOfModules);
    Eigen::VectorXd posCmd(numberOfModules);
    Eigen::VectorXd velCmd(numberOfModules);
    Eigen::VectorXd trqCmd(numberOfModules);

    if (!group->sendFeedbackRequest()) {
      return;
    }

    if (!group->getNextFeedback(feedback)) {
        return;
    }

    const size_t numberOfWaypoints = 2;
    Eigen::VectorXd time(numberOfWaypoints);
    time[0] = 0.0;
    time[1] = timeDuration;

    Eigen::MatrixXd positions(numberOfModules, numberOfWaypoints);
    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(numberOfModules, numberOfWaypoints);
    Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(numberOfModules, numberOfWaypoints);

    Eigen::VectorXd currentPosition = feedback.getPosition();
    positions.col(0) = currentPosition;
    positions.col(1) = *endPoint;

    auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocity, &torque);

    const double duration = trajectory->getDuration();
    const double period = 1.0 / static_cast<double>(group->getFeedbackFrequencyHz());

    for (double t = 0.0; t < duration; t += period) {
        group->getNextFeedback(feedback);

        trajectory->getState(t, &posCmd, &velCmd, &trqCmd);
        cmd.setPosition(posCmd);
        cmd.setVelocity(velCmd);
        Eigen::VectorXd effort = getGravCompEfforts(model, masses, feedback.getPosition(), gravity);
        cmd.setEffort(effort);
        group->sendCommand(cmd);
    }
}
static void moveThroughWayPoints(Group* group, const robot_model::RobotModel& model, const Eigen::VectorXd& masses, const Eigen::Vector3d& gravity, trajectory::Trajectory* trajectory) {
    const int numberOfModules = group->size();
    const double duration = trajectory->getDuration();

    GroupCommand cmd(numberOfModules);
    Eigen::VectorXd posCmd(numberOfModules);
    Eigen::VectorXd velCmd(numberOfModules);
    Eigen::VectorXd trqCmd(numberOfModules);

    /// NOTE: First thing to do is move the arm from its current position to the position of the first waypoint
    trajectory->getState(trajectory->getStartTime(), &posCmd, nullptr, nullptr);

    moveToWayPoint(
        group,
        model, masses, gravity,
        &posCmd,
        duration / static_cast<double>(trajectory->getWaypointCount())
        /// timeDuration = duration to move through a single waypoint in the trajectory 
        /// This way, moving to the first waypoint is at the same speed as the subsequent
        /// waypoint to waypoint motion. 
    );

    const double period = 1.0 / static_cast<double>(group->getFeedbackFrequencyHz());

    GroupFeedback feedback(group->size());
    for (double t = 0.0; t < duration; t += period) {
        group->getNextFeedback(feedback);

        trajectory->getState(t, &posCmd, &velCmd, &trqCmd);
        cmd.setPosition(posCmd);
        cmd.setVelocity(velCmd);
        Eigen::VectorXd effort = getGravCompEfforts(model, masses, feedback.getPosition(), gravity);
        cmd.setEffort(effort);
        group->sendCommand(cmd);
    }
}
    
} // namespace example
} // namespace hebi
 
void chatterCallback(const tomatobot::xyzArray::ConstPtr& msg)
{
    float x [msg->xyzCoord.size()];
	float y [msg->xyzCoord.size()];
	float z [msg->xyzCoord.size()];
	target_matrix << MatrixXf::Zero(3,20);
    for (int i=0; i<msg->xyzCoord.size(); ++i)
    {
		const tomatobot::xyz &data = msg->xyzCoord[i];
		x[i] = data.x;
		y[i] = data.y;
		z[i] = data.z;
	   
		target = true;
	}
	for (int jj = 0; jj < msg->xyzCoord.size(); jj++){
		target_matrix(0,jj) = x[jj];
		target_matrix(1,jj) = y[jj];
		target_matrix(2,jj) = z[jj];
	}
	commandSize = msg->xyzCoord.size();
}

 
int main(int argc, char* argv[]) {
    /// This is a macro that expands on Linux systems to enable behavior needed for `hebi_getchar()`
    //hebi_getchar_init();
	ros::init(argc, argv, "arm");
	ros::NodeHandle n;
	
	
    /// Modify the structure below to suit your needs.
    hebi::example::CreateDesc desc = {
        { "Base", "Shoulder", "Elbow", "Wrist" }, /// modules
        { "fourGrowers" },                     /// families - Note that you can...
                                         /// use 1 family if all modules are part of the same family

        { 0, 2.44, 1.81, -0.6 },         /// home waypoint
        100,                             /// command lifetime (in ms)
        .5,                             /// time to move from one waypoint to the next
        500.0                            /// Feedback frequency
    };

    std::shared_ptr<hebi::Group> group;

    if (!hebi::example::createGroup(desc, group)) {
        std::cerr << "Could not get group" << std::endl;
        return 1;
    }

    auto numberOfModules = group->size();
    Eigen::VectorXd homePosition(numberOfModules);

    /// Populate home position vector
    for (int i = 0; i < numberOfModules; i++) {
        homePosition[i] = desc.homePosition[i];
    }


    // Create robot model and get masses/gravity for grav-comp code
    robot_model::RobotModel model;
    model.addActuator(ActuatorType::X5_9);
    model.addBracket(BracketType::X5HeavyRightOutside);
    model.addActuator(ActuatorType::X5_9);
    model.addLink(LinkType::X5, 0.275, M_PI);
    model.addActuator(ActuatorType::X5_9);
    model.addLink(LinkType::X5, 0.275, M_PI);
    model.addActuator(ActuatorType::X5_4);
   // model.addLink(LinkType::X5, 0.150, 0);
   model.addLink(LinkType::X5, 0.125, 0);
  
    Eigen::VectorXd masses(model.getFrameCount(HebiFrameTypeCenterOfMass));
    model.getMasses(masses);
    Eigen::Vector3d gravity(0, 0, -1);

	Eigen::Vector3d actual_xyz;
	Eigen::Vector3d error_xyz;
	Eigen::VectorXd initial_joint_angles(group->size());
	Eigen::VectorXd ik_result_joint_angles(group->size());
	//Setting Joint angles to home position	
	initial_joint_angles = homePosition;
	  
	// Set joint limits to force a particular solution (elbow up, in this case)
	Eigen::VectorXd min_positions(group->size());
	min_positions << -0.25, 0, 0, -1;
	Eigen::VectorXd max_positions(group->size());
	max_positions << 0.25, M_PI, M_PI, 1;
	
	

	
	bool command_sent = true;
	bool exit_command = false;
	bool position_set = true;
	Eigen::VectorXd currentPosition(numberOfModules);
	std::thread t1 {[&](){
		const int numberOfModules = group->size();
		GroupFeedback feedback(group->size());
		GroupCommand cmd(numberOfModules);
		Eigen::VectorXd posCmd(numberOfModules);
		Eigen::VectorXd velCmd(numberOfModules);
		Eigen::VectorXd trqCmd(numberOfModules);
		
		while(!exit_command){
			if(!position_set){
				//group->getNextFeedback(feedback);
				//currentPosition = feedback.getPosition();
				currentPosition = homePosition;
				position_set = true;
		    }
			while(!command_sent){
				group->getNextFeedback(feedback);
				cmd.setPosition(currentPosition);
				Eigen::VectorXd effort = getGravCompEfforts(model, masses, feedback.getPosition(), gravity);
				cmd.setEffort(effort);
				group->sendCommand(cmd);
			}
		}
		std::cerr << "Exiting Thread" << std::endl;
	}};
	
	target = false;
	bool perform_action;
	int iteration = 0;
	while(true){
		//Getting Target Location
		//getInput(target_xyz);
		
		
		ros::Subscriber sub = n.subscribe("pathplan", 2, chatterCallback);
		//For ROS Integration//
		while(!target){
			ros::spinOnce();
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		
		std::cerr << "Target Set" << std::endl;
		std::cerr << "Command Size: " << commandSize << std::endl << std::endl;
		Eigen::MatrixXd traj_matrix(4,commandSize);
		//traj_matrix.col(0) = initial_joint_angles;
		int count = 0;
		for (int zz = 0; zz < commandSize; zz++){
			float x_trim = target_matrix(0,zz);
			float y_trim = target_matrix(1,zz);
			float z_trim = target_matrix(2,zz);
			
			target_xyz << x_trim, y_trim, z_trim;
				
			if((target_xyz(0) != 0) || (target_xyz(1) != 0) || (target_xyz(2) != 0)){
				model.solveIK(
					initial_joint_angles,
					ik_result_joint_angles,
					robot_model::EndEffectorPositionObjective(target_xyz),
					robot_model::JointLimitConstraint(min_positions, max_positions)
				);
		}
						
			Eigen::Matrix4d transform;
			std::cerr << "Target position: " << std::endl << target_xyz.transpose() << std::endl;
			std::cerr << "IK joint angles: " << std::endl << ik_result_joint_angles.transpose() << std::endl;
			model.getEndEffector(ik_result_joint_angles, transform);
			actual_xyz << transform.topRightCorner<3,1>();
			std::cerr << "FK1 of IK joint angles: " <<std::endl << actual_xyz.transpose() << std::endl << std::endl;
			
			error_xyz << (actual_xyz - target_xyz);
			error_xyz << error_xyz.array().abs();
			std::cerr << "Error: " << error_xyz.transpose() << std::endl;
			
			perform_action = true;
			if (error_xyz(0) > ERROR_THRESH || error_xyz(1) > ERROR_THRESH || error_xyz(2) > ERROR_THRESH){
				std::cerr << "The Error is too Large...Not Performing Action" <<std::endl << std::endl;
				//perform_action = false;
			}
			else {
			traj_matrix.col(count) = ik_result_joint_angles;
			initial_joint_angles = ik_result_joint_angles;
			std::cerr << "traj_matrix: " << traj_matrix.transpose() << std::endl;
			count++;
			}
		}
			
			
			//////////////////////////////////////
			// Send commands to the physical robot
			//////////////////////////////////////
			if(perform_action){
				//Eigen::MatrixXd positions(group->size(),3);
				//positions << initial_joint_angles[0], ik_result_joint_angles[0], ik_result_joint_angles2[0], 
					   //initial_joint_angles[1], ik_result_joint_angles[1], ik_result_joint_angles2[1], 
					   //initial_joint_angles[2], ik_result_joint_angles[2], ik_result_joint_angles2[2], 
					   //initial_joint_angles[3], ik_result_joint_angles[3], ik_result_joint_angles2[3];
					   
				//Eigen::MatrixXd positions(group->size(),4);
				//positions << initial_joint_angles[0], ik_result_joint_angles[0], ik_result_joint_angles2[0], ik_result_joint_angles2[0],
					   //initial_joint_angles[1], ik_result_joint_angles[1], ik_result_joint_angles2[1], ik_result_joint_angles2[1],
					   //initial_joint_angles[2], ik_result_joint_angles[2], ik_result_joint_angles2[2], ik_result_joint_angles2[2],
					   //initial_joint_angles[3], ik_result_joint_angles[3], ik_result_joint_angles2[3], ik_result_joint_angles2[3];
						   
					  
				//The times to reach each waypoint (in seconds)
				Eigen::VectorXd time(count+2);
				double scale = 0;
				double calibration = 0;
				time(0,0) = 0;
				time(1,0) = 1 + scale;
				double timeSum = time(1.0);
				for (int zz = 2; zz <= count+1; zz++){
					if(zz == count + 1){
						time(zz,0) = 1 + timeSum + scale;
						timeSum = timeSum + 1 + scale;
					}
					else if(zz % 3 == 2){
						time(zz,0) = 0.5 + timeSum + scale;
						timeSum = timeSum + 0.5 + scale;
					}
					else if(zz % 3 == 0){
						time(zz,0) = 0.25 + timeSum + scale + calibration;
						timeSum = timeSum + 0.25 + scale + calibration;
					}
					else{
						time(zz,0) = 0.5 + timeSum + scale;
						timeSum = timeSum + 0.5 + scale;
					}
				}
				if(count > 0){
					//Eigen::VectorXd time(2);
					std::cerr << "Count: " << count << std::endl;
					std::cerr << "time: " << time.transpose() << std::endl;

					traj_matrix.conservativeResize(traj_matrix.rows(),count);
					Eigen::MatrixXd temp(traj_matrix.rows(), traj_matrix.cols()+1);
					Eigen::MatrixXd temp2(traj_matrix.rows(), traj_matrix.cols()+2);
					temp.leftCols<1>(1) = homePosition; //Setting Initial Home Position

					std::cerr << "Temp w/ Home: " << temp << std::endl;

					temp2.rightCols<1>(1) = homePosition; //End Home Positions
					temp.bottomRightCorner(traj_matrix.rows(),traj_matrix.cols()) = traj_matrix;
					std::cerr << "Temp w/ Swap: " << temp << std::endl << std::endl;
					temp2.topLeftCorner(temp.rows(),temp.cols()) = temp;
					std::cerr << "Temp2 w/ Homes: " << temp2 << std::endl << std::endl;
					traj_matrix.swap(temp2);
					std::cerr << "Traj_Matrix Altered: " << traj_matrix.transpose() << std::endl;
					Eigen::MatrixXd vel_constraints = Eigen::MatrixXd::Zero(traj_matrix.rows(), traj_matrix.cols());
					Eigen::MatrixXd accel_constraints = Eigen::MatrixXd::Zero(traj_matrix.rows(),traj_matrix.cols());
					auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, traj_matrix, &vel_constraints, &accel_constraints);
					
					std::cerr << "Trajectory: " << trajectory <<std::endl;
						 
					if (trajectory != 0){
						command_sent = true;
						/// Move to home position
						//if (iteration == 0){
							//std::cerr << "Sending Command" << std::endl;
							//hebi::example::moveToWayPoint(group.get(), model, masses, gravity, &homePosition, 1.5);
						//}
						
						
						std::cerr << "Moving through waypoints" << std::endl;
						/// Move through waypoints
						hebi::example::moveThroughWayPoints(group.get(), model, masses, gravity, trajectory.get());

						///// Move to home position
						////hebi::example::moveToWayPoint(group.get(), model, masses, gravity, &homePosition, desc.waypointTransitionTime);
						//hebi::example::moveToWayPoint(group.get(), model, masses, gravity, &homePosition, 1.5);
						position_set = false;
						command_sent = false;
						std::cerr << "Just Finished" << std::endl << std::endl;
					}
					else{
						std::cerr << "Trajectory Equaled Zero" << std::endl;
					}
				}

				else{
					std::cerr << "There were no coordinates" << std::endl;
				}

				iteration++;
			}
				
		else {
			std::cerr << "Eigen was 0,0,0" << std::endl;
			std::cerr << "Exiting Now" << std::endl;
			exit_command = true;
			command_sent = true;
			target = false;
			break;			
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		target = false;
	}
	t1.join();
    return 0;
}
