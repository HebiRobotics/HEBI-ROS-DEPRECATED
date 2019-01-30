#include <ros/ros.h>

#include <gtest/gtest.h>

#include <thread>

#include "hebiros.h"
#include "hebiros/AddModelFromURDFSrv.h"
#include "hebiros/ModelFkSrv.h"

class FKTests6DOF : public ::testing::Test {
  public:
    FKTests6DOF() {
    }
    ~FKTests6DOF() {}

    static void AddModel() {
      ros::NodeHandle n;

      ros::ServiceClient add_model_client = n.serviceClient<hebiros::AddModelFromURDFSrv>(
        "/hebiros/add_model_from_urdf");
      
      hebiros::AddModelFromURDFSrv add_model_srv;

      add_model_srv.request.model_name = model_name;
      add_model_srv.request.description_param = "6_dof_robot_description";

      add_model_client.call(add_model_srv);
    }
  protected:
    // 6 DOF tests have 6 modules!!!
    const size_t num_modules = 6;

    static const std::string model_name;
      
    ros::NodeHandle n;

    // Compare the results to those generated from MATLAB ground truth
    void CompareResults(double* matlab, double* ros) {
      // Note: we ignore duplicated "actuator internal" frames from the ROS API
      std::vector<size_t> ignored_frames = { 0, 1, 4, 7, 10, 13, 16 };
      auto search_start = ignored_frames.begin();
      size_t matlab_frame = 0;
      size_t ros_frame = 0;

      while (matlab_frame < 11 && ros_frame < 17) {
        auto found = std::find(search_start, ignored_frames.end(), ros_frame);
        if (found != ignored_frames.end()) {
          search_start = found; // A tiny bit more efficient -- don't look through the starting elements
          ++ros_frame;
          continue;
        }

        // Note: ML data is column major, ROS is row major
        for (size_t i = 0; i < 3; ++i)
          for (size_t j = 0; j < 3; ++j)
            ASSERT_NEAR(matlab[matlab_frame*16 + i*4 + j], ros[ros_frame*16 + j*4 + i], 0.001);
        ++ros_frame;
        ++matlab_frame;
      }
    }
};

const std::string FKTests6DOF::model_name { "6DOF" };

// Note: the MATLAB ground truth is from the following script.
// unfortunately, reshape gives column-major reordering :(
//
// kin = HebiKinematics
// kin.addBody('X8-9');
// kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
// kin.addBody('X8-9', 'PosLim', [-0.6 1.2]); % gas spring limits
// kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
// kin.addBody('X8-9');
// kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
// kin.addBody('X5-1');
// kin.addBody('X5-LightBracket', 'mount', 'right');
// kin.addBody('X5-1');
// kin.addBody('X5-LightBracket', 'mount', 'right');
// kin.addBody('X5-1');
// reshape(kin.getFK('Output', <angle vector goes here>), 1, 11 * 16);
TEST_F(FKTests6DOF, GetOutputFrames) {
  ros::ServiceClient model_fk_client = n.serviceClient<hebiros::ModelFkSrv>(
    std::string("/hebiros/") + model_name + std::string("/fk"));
      
  hebiros::ModelFkSrv model_fk_srv;

  model_fk_srv.request.positions.resize(num_modules);

  // First, try with a vector of zeros
  for (size_t i = 0; i < num_modules; ++i)
    model_fk_srv.request.positions[i] = 0;
  model_fk_srv.request.frame_type = hebiros::ModelFkSrv::Request::FrameTypeOutput;
  ASSERT_TRUE(model_fk_client.call(model_fk_srv));
  double ml[176] = {
     1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.0451, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0,-1, 0, 0, 0,-0.0375, 0.11, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0,-1, 0, 0, 0,-0.0826, 0.11, 1, 1, 0, 0, 0, 0, 0,-1, 0, 0, 1, 0, 0, 0.3250,-0.0826, 0.11, 1, 1, 0, 0, 0, 0, 0,-1, 0, 0, 1, 0, 0, 0.3250,-0.0375, 0.11, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0,-1, 0, 0, 0.6500,-0.0375, 0.11, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0,-1, 0, 0, 0.6500,-0.0685, 0.11, 1, 1, 0, 0, 0, 0,-1, 0, 0, 0, 0,-1, 0, 0.6500,-0.1086, 0.0571, 1, 1, 0, 0, 0, 0,-1, 0, 0, 0, 0,-1, 0, 0.6500,-0.1086, 0.0261, 1, 1, 0, 0, 0, 0, 0,-1, 0, 0, 1, 0, 0, 0.6500,-0.0656,-0.01, 1, 1, 0, 0, 0, 0, 0,-1, 0, 0, 1, 0, 0, 0.6500,-0.0345,-0.01, 1 };
  CompareResults(ml, model_fk_srv.response.frames.data());

  // Some random angles:
  model_fk_srv.request.positions = {.325, 12.22, 42.4, .033, .2352, 31.34};
  ASSERT_TRUE(model_fk_client.call(model_fk_srv));
  double ml2[176] = {
 0.9477, 0.3193, 0, 0,-0.3193, 0.9477, 0, 0, 0, 0, 1, 0, 0, 0, 0.0451, 1, 0.9477, 0.3193, 0, 0,-0, 0, 1, 0, 0.3193,-0.9477, 0, 0, 0.0120,-0.0355, 0.1001, 1, 0.8914, 0.3003,-0.3395, 0, 0.3217, 0.1084, 0.9406, 0, 0.3193,-0.9477, 0, 0, 0.0264,-0.0783, 0.1001, 1, 0.8914, 0.3003,-0.3395, 0,-0.3217,-0.1084,-0.9406, 0,-0.3193, 0.9477,-0, 0, 0.3161, 0.0193,-0.0102, 1, 0.3114, 0.1049, 0.9445, 0, 0.8950, 0.3016,-0.3286, 0,-0.3193, 0.9477,-0, 0, 0.3017, 0.0621,-0.0102, 1, 0.3114, 0.1049, 0.9445, 0,-0.8950,-0.3016, 0.3286, 0, 0.3193,-0.9477, 0, 0, 0.4029, 0.0962, 0.2967, 1, 0.2817, 0.0949, 0.9548, 0,-0.9048,-0.3049, 0.2973, 0, 0.3193,-0.9477, 0, 0, 0.4128, 0.0668, 0.2967, 1, 0.2817, 0.0949, 0.9548, 0, 0.3193,-0.9477, 0, 0, 0.9048, 0.3049,-0.2973, 0, 0.4645, 0.0420, 0.2839, 1, 0.3484,-0.1285, 0.9285, 0, 0.2449,-0.9437,-0.2225, 0, 0.9048, 0.3049,-0.2973, 0, 0.4926, 0.0514, 0.2747, 1, 0.3484,-0.1285, 0.9285, 0, 0.9048, 0.3049,-0.2973, 0,-0.2449, 0.9437, 0.2225, 0, 0.5182, 0.1042, 0.2724, 1, 0.2788,-0.1513, 0.9484, 0, 0.9286, 0.2942,-0.2260, 0,-0.2449, 0.9437, 0.2225, 0, 0.5106, 0.1335, 0.2793, 1 };
  CompareResults(ml2, model_fk_srv.response.frames.data());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "FKTests6DOFNode");
  
  testing::InitGoogleTest(&argc, argv);
  
  std::thread t([]{while(ros::ok()) ros::spin();});
 
  // This should only be run once, as it changes the state of the node itself
  // (we should probably test the add model code separately, fwiw) 
  FKTests6DOF::AddModel();
 
  auto res = RUN_ALL_TESTS();
  
  ros::shutdown();
  
  return res;
}
