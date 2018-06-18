#include "hebiros_model.h"

#include "hebiros.h"

std::map<std::string, HebirosModel> HebirosModel::models;

bool HebirosModel::load(const std::string& name, const std::string& description_param) {

  // Try to load:
  urdf::Model model;
  bool success = loadURDF(description_param, model);
  if (!success)
    return false;

  std::unique_ptr<hebi::robot_model::RobotModel> robot_model = parseURDF(model);
  if (!robot_model)
    return false;

  models.emplace(name, std::move(robot_model));
  return true;
}


HebirosModel::HebirosModel(std::unique_ptr<hebi::robot_model::RobotModel> model_)
 : model(std::move(model_)) {
}

HebirosModel* HebirosModel::getModel(const std::string& model_name) {
  if (models.count(model_name) == 0)
    return nullptr;
  return &models.at(model_name);
}

hebi::robot_model::RobotModel& HebirosModel::getModel() {
  return *model;
}

bool HebirosModel::loadURDF(const std::string& description_param, urdf::Model& model) {

  if (!model.initParam(description_param)) {
    ROS_WARN_STREAM("Could not load " << description_param);
    return false;
  }

  ROS_INFO_STREAM("Loaded URDF from " << description_param);
  return true;
}
    
class UnsupportedStructureException : public std::exception {
  public:
    UnsupportedStructureException(const std::string& structure_type) :
      message_("Structures of type " + structure_type + " not yet supported.") {
    }
    const char* what() const noexcept override {
      return message_.c_str();
    }
  private:
    std::string message_;
};

class UnsupportedJointException : public std::exception {
  public:
    UnsupportedJointException(const std::string& joint_type) :
      message_("Joints of type " + joint_type + " not yet supported.") {
    }
    const char* what() const noexcept override {
      return message_.c_str();
    }
  private:
    std::string message_;
};

Eigen::MatrixXd ROSPoseToEigenMatrix(const urdf::Pose& pose) {
  Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  res.topLeftCorner<3,3>() = q.toRotationMatrix();
  res(0, 3) = pose.position.x;
  res(1, 3) = pose.position.y;
  res(2, 3) = pose.position.z;
  return res;
}

void parseInner(const urdf::Link& link, hebi::robot_model::RobotModel& model) {
  Eigen::Matrix4d com = Eigen::Matrix4d::Identity();
  Eigen::VectorXd inertia;
  inertia.resize(6);
  inertia << 0, 0, 0, 0, 0, 0;
  float mass = 0;

  // Add details about this link:
  auto& inertial = link.inertial;
  if (inertial) {
    com = ROSPoseToEigenMatrix(inertial->origin);
    inertia <<
      inertial->ixx, inertial->iyy, inertial->izz,
      inertial->ixy, inertial->ixz, inertial->iyz;
    mass = inertial->mass;
  }

  auto& child_links = link.child_links;

  // TODO: support multi-output elements once supported in the C/C++ HEBI API;
  // the below code would then iterate through the child links
  if (child_links.size() > 1)
    throw UnsupportedStructureException("tree");

  if (child_links.size() == 0) {
    // TODO: provide some smart default for leaf nodes (e.g., COM * 2)? 
    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
    model.addRigidBody(com, inertia, mass, output, false);
    return;
  }

  // hebi robot model "outputs" depends on where the children connect.
  // (assume one output here for now; can support trees later)
  {
    auto& child_link = child_links[0];
    auto& child_joint = child_link->parent_joint;
    Eigen::Matrix4d output =
      ROSPoseToEigenMatrix(child_joint->parent_to_joint_origin_transform);

    // Add rigid body for this link
    model.addRigidBody(com, inertia, mass, output, false);
  }

  // Add joint(s) to children, and child links (this is written to support
  // trees)
  for (auto& child_link : child_links) {
    auto& child_joint = child_link->parent_joint;
    HebiJointType joint_type = HebiJointTypeRotationX;
    // Add joint and link:
    if (child_joint->type == urdf::Joint::REVOLUTE | child_joint->type == urdf::Joint::CONTINUOUS) {
      if (child_joint->axis.x != 0) {
        if (child_joint->axis.y != 0 && child_joint->axis.z != 0) {
          throw UnsupportedJointException("non-principal rotation axis");
        }
        joint_type = HebiJointTypeRotationX;
      } else if (child_joint->axis.y != 0) {
        if (child_joint->axis.z != 0) { // We know x == 0
          throw UnsupportedJointException("non-principal rotation axis");
        }
        joint_type = HebiJointTypeRotationY;
      } else if (child_joint->axis.z != 0) {
        joint_type = HebiJointTypeRotationZ; // We know x == 0 && y == 0
      }
      model.addJoint(joint_type, false);
    } else if (child_joint->type == urdf::Joint::PRISMATIC) {
      if (child_joint->axis.x != 0) {
        if (child_joint->axis.y != 0 && child_joint->axis.z != 0) {
          throw UnsupportedJointException("non-principal translation axis");
        }
        joint_type = HebiJointTypeTranslationX;
      } else if (child_joint->axis.y != 0) {
        if (child_joint->axis.z != 0) { // We know x == 0
          throw UnsupportedJointException("non-principal translation axis");
        }
        joint_type = HebiJointTypeTranslationY;
      } else if (child_joint->axis.z != 0) {
        joint_type = HebiJointTypeTranslationZ; // We know x == 0 && y == 0
      }
      model.addJoint(joint_type, false);
    } else if (child_joint->type == urdf::Joint::FIXED) {
      // This is supported, we just don't need a "binding" joint in the hebi
      // robot model classes
    } else {
      throw UnsupportedJointException("unknown");
    }
    // This recursively adds this child link (and its children)
    parseInner(*child_link, model);
  } 
}

std::unique_ptr<hebi::robot_model::RobotModel> HebirosModel::parseURDF(const urdf::Model& model)
{
  const urdf::Link* root = model.getRoot().get();
  std::unique_ptr<hebi::robot_model::RobotModel> rm(new hebi::robot_model::RobotModel());

  // Note: in the future, we could support selection of specific child links
  // from the URDF file.
  try {
    auto& children = root->child_links;
    if (children.size() != 1) {
      throw UnsupportedStructureException("more or less than a single model in the world");
    }
    auto& child = children[0];
    if (child->parent_joint->type != urdf::Joint::FIXED) {
      throw UnsupportedStructureException("parent joint not connected via a fixed joint");
    }

    // Set base frame according to this object
    rm->setBaseFrame(ROSPoseToEigenMatrix(child->parent_joint->parent_to_joint_origin_transform));

    parseInner(*child, *rm.get());
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(std::string(e.what()));
    return nullptr;
  }

  return rm;
}
