#include "Initialization.h"
#include <yaml-cpp/yaml.h>

namespace clear {

Initialization::Initialization(Node::SharedPtr nodeHandle,
                               std::string config_yaml)
    : nodeHandle_(nodeHandle), config_yaml_(config_yaml) {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  reset_state_client_ = nodeHandle_->create_client<trans::srv::SimulationReset>(
      topic_prefix + "sim_reset");
}

Initialization::~Initialization() {}

void Initialization::reset_simulation() {
  auto config_ = YAML::LoadFile(config_yaml_);
  const auto joints_name = config_["model"]["actuated_joints_name"]
                               .as<std::vector<std::string>>();
  const auto joint_pos =
      config_["model"]["default"]["joint_pos"].as<std::vector<scalar_t>>();
  const auto base_pos =
      config_["model"]["default"]["base_pos"].as<std::vector<scalar_t>>();
  const auto base_quat =
      config_["model"]["default"]["base_quat"].as<std::vector<scalar_t>>();

  auto request = std::make_shared<trans::srv::SimulationReset::Request>();
  request->header.frame_id = config_["model"]["name"].as<std::string>();
  request->base_pose.orientation.w = base_quat[0];
  request->base_pose.orientation.x = base_quat[1];
  request->base_pose.orientation.y = base_quat[2];
  request->base_pose.orientation.z = base_quat[3];
  request->base_pose.position.x = base_pos[0];
  request->base_pose.position.y = base_pos[1];
  request->base_pose.position.z = base_pos[2];

  request->joint_state.name = joints_name;
  for (int i = 0; i < joints_name.size(); i++) {
    request->joint_state.position.push_back(joint_pos[i]);
  }

  RCLCPP_INFO(nodeHandle_->get_logger(), "waiting for service %s ...",
              reset_state_client_->get_service_name());
  while (!reset_state_client_->wait_for_service(20ms)) {
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(nodeHandle_->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
  }

  auto result = reset_state_client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(nodeHandle_, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    if (result.get()->is_success) {
      RCLCPP_INFO(nodeHandle_->get_logger(),
                  "call service reset_state success");
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to reset state");
    }
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(),
                 "Failed to call service reset_state");
  }
}

} // namespace clear