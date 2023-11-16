#include <MotionManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

MotionManager::MotionManager(std::string config_yaml)
    : Node("MotionManager"), config_yaml_(config_yaml) {}

MotionManager::~MotionManager() {
  run_.push(false);
  inner_loop_thread_.join();
}

void MotionManager::init() {
  intializationPtr_ =
      std::make_shared<Initialization>(this->shared_from_this(), config_yaml_);

  intializationPtr_->reset_simulation();
  rclcpp::spin_some(this->shared_from_this());

  estimatorPtr_ = std::make_shared<StateEstimationLKF>(this->shared_from_this(),
                                                       config_yaml_);
  trajGenPtr_ = std::make_shared<TrajectorGeneration>(this->shared_from_this(),
                                                      config_yaml_);
  atscImplPtr_ =
      std::make_shared<AtscImpl>(this->shared_from_this(), config_yaml_,
                                 trajGenPtr_->get_robot_interface());

  auto config_ = YAML::LoadFile(config_yaml_);
  bool hardware_ = config_["estimation"]["hardware"].as<bool>();
  if (hardware_) {
    unitreeHWPtr_ =
        std::make_shared<UnitreeHW>(this->shared_from_this(), config_yaml_);
  }

  inner_loop_thread_ = std::thread(&MotionManager::inner_loop, this);
  run_.push(true);
}

void MotionManager::inner_loop() {
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  rclcpp::Rate loop_rate(500.0);
  const scalar_t ts = this->now().seconds();
  while (rclcpp::ok() && run_.get()) {
    if (this->now().seconds() > ts + 4.0) {
      trajGenPtr_->setVelCmd(vector3_t(0.0, 0.0, 0.0), 0.0);
    }
    // if (gaitSchedulePtr_->get_current_gait_name() == "trot") {
    //   trajGenPtr_->setVelCmd(vector3_t(0.0, 0.0, 0.0), 0.0);
    // } else {
    //   trajGenPtr_->setVelCmd(vector3_t(0.0, 0.0, 0.0), 0.0);
    // }
    if (unitreeHWPtr_ != nullptr) {
      unitreeHWPtr_->read();
      estimatorPtr_->set_imu_msg(unitreeHWPtr_->get_imu_msg());
      estimatorPtr_->set_touch_msg(unitreeHWPtr_->get_touch_msg());
      estimatorPtr_->set_joint_msg(unitreeHWPtr_->get_joint_msg());
    }

    trajGenPtr_->update_current_state(estimatorPtr_->getQpos(),
                                      estimatorPtr_->getQvel());

    atscImplPtr_->update_current_state(estimatorPtr_->getQpos(),
                                       estimatorPtr_->getQvel());

    atscImplPtr_->update_mpc_solution(trajGenPtr_->get_mpc_sol());

    // if (unitreeHWPtr_ != nullptr) {
    //   unitreeHWPtr_->set_actuator_cmds(atscImplPtr_->getCmds());
    //   unitreeHWPtr_->send();
    // }

    loop_rate.sleep();
  }
}

} // namespace clear
