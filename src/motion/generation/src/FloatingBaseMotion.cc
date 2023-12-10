#include "generation/FloatingBaseMotion.h"
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

FloatingBaseMotion::FloatingBaseMotion(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
    std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      refTrajBuffer_(referenceTrajectoriesBuffer) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);

  base_name = config_["model"]["base_name"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("FloatingBaseMotion"), "base_name: %s",
              base_name.c_str());
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();

  lipm_ptr_ = std::make_shared<LinearInvertedPendulum>(
      nodeHandle_, pinocchioInterface_ptr_, refTrajBuffer_);
}

FloatingBaseMotion::~FloatingBaseMotion() {}

void FloatingBaseMotion::generate() {
  // generate_reference();
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("FloatingBaseMotion"), "generate base motion");
  lipm_ptr_->optimize();
}

} // namespace clear
