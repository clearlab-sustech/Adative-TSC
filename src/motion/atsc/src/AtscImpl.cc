#include "atsc/AtscImpl.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <fstream>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

AtscImpl::AtscImpl(Node::SharedPtr nodeHandle, const std::string config_yaml,
                   std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
                       robot_interface_ptr)
    : nodeHandle_(nodeHandle), robot_interface_ptr_(robot_interface_ptr) {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string actuators_cmds_topic =
      config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
  freq_ = config_["controller"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "frequency: %f", freq_);
  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();
  robot_name = config_["model"]["name"].as<std::string>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
  actuators_cmds_pub_ptr_ =
      nodeHandle_->create_publisher<trans::msg::ActuatorCmds>(
          topic_prefix + actuators_cmds_topic, qos);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "model file: %s", urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  auto foot_names =
      config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "foot name: %s", name.c_str());
  }
  pinocchioInterface_ptr_->setContactPoints(foot_names);

  base_name = config_["model"]["base_name"].as<std::string>();

  run_.push(true);
  wbcPtr_ = std::make_shared<WholeBodyController>(nodeHandle_, config_yaml,
                                                  robot_interface_ptr_);
  inner_loop_thread_ = std::thread(&AtscImpl::inner_loop, this);
}

AtscImpl::~AtscImpl() {
  run_.push(false);
  inner_loop_thread_.join();
}

void AtscImpl::update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                                    std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void AtscImpl::update_mpc_solution(
    std::shared_ptr<ocs2::PrimalSolution> mpc_sol) {
  mpc_sol_buffer.push(mpc_sol);
}

trans::msg::ActuatorCmds::SharedPtr AtscImpl::getCmds() {
  if (actuator_commands_.get() == nullptr) {
    return nullptr;
  }

  const auto &model = pinocchioInterface_ptr_->getModel();
  trans::msg::ActuatorCmds::SharedPtr msg =
      std::make_shared<trans::msg::ActuatorCmds>();
  msg->header.frame_id = robot_name;
  msg->header.stamp = nodeHandle_->now();
  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg->names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg->gain_p.emplace_back(actuator_commands_->Kp(id));
      msg->pos_des.emplace_back(actuator_commands_->pos(id));
      msg->gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg->vel_des.emplace_back(actuator_commands_->vel(id));
      msg->feedforward_torque.emplace_back(actuator_commands_->torque(id));
    }
  }
  return msg;
}

void AtscImpl::publishCmds() {
  const auto &model = pinocchioInterface_ptr_->getModel();

  trans::msg::ActuatorCmds msg;
  msg.header.frame_id = robot_name;
  msg.header.stamp = nodeHandle_->now();

  if (actuator_commands_.get() == nullptr) {
    return;
  }

  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg.names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg.gain_p.emplace_back(actuator_commands_->Kp(id));
      msg.pos_des.emplace_back(actuator_commands_->pos(id));
      msg.gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg.vel_des.emplace_back(actuator_commands_->vel(id));
      msg.feedforward_torque.emplace_back(actuator_commands_->torque(id));
    }
  }
  actuators_cmds_pub_ptr_->publish(msg);
}

void AtscImpl::inner_loop() {

  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        mpc_sol_buffer.get().get() == nullptr) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int64_t(1000 / freq_)));
    } else {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      wbcPtr_->update_state(qpos_ptr, qvel_ptr);
      wbcPtr_->update_mpc_sol(mpc_sol_buffer.get());
      // wbcPtr_->optimize();
      actuator_commands_ = wbcPtr_->optimize();

      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(
      nodeHandle_->get_logger(), "TSC: max time %f ms,  average time %f ms",
      timer_.getMaxIntervalInMilliseconds(), timer_.getAverageInMilliseconds());
}
} // namespace clear
