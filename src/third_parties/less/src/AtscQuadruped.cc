#include "less/AtscQuadruped.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {
AtscQuadruped::AtscQuadruped(const std::string config_yaml,
                             rclcpp::Logger logger) {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(logger, "[AtscQuadruped] model file: %s", urdf.c_str());

  pinocchioInterface_ptr = std::make_shared<PinocchioInterface>(urdf.c_str());

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(logger, "[AtscQuadruped] foot name: %s", name.c_str());
  }

  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();

  auto model_ = pinocchioInterface_ptr->getModel();
  for (const auto &name : actuated_joints_name) {
    RCLCPP_INFO(logger, "[AtscQuadruped] actuated_joints name: %s",
                name.c_str());
    if (!model_.existJointName(name)) {
      throw std::runtime_error("actuated_joints " + name +
                               " dose not exist in URDF");
    }
  }
}

AtscQuadruped::~AtscQuadruped() {}

void AtscQuadruped::update_state(
    const nav_msgs::msg::Odometry &odom, // vel in local coordinate
    const sensor_msgs::msg::JointState &joints_state) {
  const pin::Model &model_ = pinocchioInterface_ptr->getModel();

  vector_t qpos = vector_t::Zero(model_.nq);
  vector_t qvel = vector_t::Zero(model_.nv);

  for (size_t k = 0; k < joints_state.name.size(); k++) {
    if (static_cast<int>(k) < model_.njoints &&
        model_.existJointName(joints_state.name[k])) {
      pin::Index id = model_.getJointId(joints_state.name[k]) - 2;
      qpos[id + 7] = joints_state.position[k];
      qvel[id + 6] = joints_state.velocity[k];
    }
  }

  const auto &orientation = odom.pose.pose.orientation;
  const auto &position = odom.pose.pose.position;
  const auto &vel = odom.twist.twist.linear;
  const auto &ang_vel = odom.twist.twist.angular;

  Eigen::Quaternion<scalar_t> quat(orientation.w, orientation.x, orientation.y,
                                   orientation.z);
  matrix3_t rot = quat.toRotationMatrix();
  qpos.head(3) << position.x, position.y, position.z;
  qvel.head(3) << vel.x, vel.y, vel.z;
  qvel.head(3) = rot.transpose() * qvel.head(3);
  qpos.segment(3, 4) << orientation.x, orientation.y, orientation.z,
      orientation.w;
  qvel.segment(3, 3) << ang_vel.x, ang_vel.y, ang_vel.z;

  pinocchioInterface_ptr->updateRobotState(qpos, qvel);
}

void AtscQuadruped::eval(std::vector<bool> torch_flag) {
  auto cmds = std::make_shared<ActuatorCmds>();
  const size_t na = actuated_joints_name.size();
  cmds->names = actuated_joints_name;
  cmds->Kp.setZero(na);
  cmds->Kd.setZero(na);
  cmds->pos.setZero(na);
  cmds->vel.setZero(na);
  cmds->tau.setZero(na);

  auto model_ = pinocchioInterface_ptr->getModel();
  vector_t gcmp = pinocchioInterface_ptr->nle();
  for (size_t i = 0; i < na; i++) {
    pin::Index id = model_.getJointId(actuated_joints_name[i]) - 2;
    cmds->tau(i) = gcmp(id + 6);
  }
  actuator_cmds_buffer_.push(cmds);
}

std::shared_ptr<const AtscQuadruped::ActuatorCmds>
AtscQuadruped::getActuatorCmds() {
  return actuator_cmds_buffer_.get();
}

Task AtscQuadruped::getNewtonEulerEquation() {
  Task t;
  return t;
}

Task AtscQuadruped::getMaintainContactTask() {
  Task t;
  return t;
}

Task AtscQuadruped::getFrictionCone() {
  Task t;
  return t;
}

Task AtscQuadruped::getTorqueLimits() {
  Task t;
  return t;
}

Task AtscQuadruped::getFloatingBaseTask() {
  Task t;
  return t;
}

Task AtscQuadruped::getSwingTask() {
  Task t;
  return t;
}

} // namespace clear
