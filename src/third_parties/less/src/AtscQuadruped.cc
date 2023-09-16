#include "less/AtscQuadruped.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {
AtscQuadruped::AtscQuadruped(const std::string config_yaml) {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string model_package =
      config_["controller"]["model_package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["controller"]["urdf"].as<std::string>();
  printf("[AtscQuadruped] model file: %s", urdf.c_str());

  pinocchioInterface_ptr = std::make_shared<PinocchioInterface>(urdf.c_str());

  newton_euler_eq_ptr =
      make_shared<NewtonEulerEquation>(pinocchioInterface_ptr);
  contact_ptr = make_shared<MaintainContactTask>(pinocchioInterface_ptr);
  friction_cone_ptr = make_shared<FrictionCone>(pinocchioInterface_ptr);
  torque_limits_ptr = make_shared<TorqueLimits>(pinocchioInterface_ptr);
  se3_array_ptr.emplace_back(
      std::make_shared<SE3Task>(pinocchioInterface_ptr, "floating_base"));
  auto foot_names =
      config_["controller"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    printf("[AtscQuadruped] foot name: %s", name.c_str());
    se3_array_ptr.emplace_back(
        std::make_shared<SE3Task>(pinocchioInterface_ptr, name));
  }
}

AtscQuadruped::~AtscQuadruped() {}

} // namespace clear
