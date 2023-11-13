#include "generation/TrajectorGeneration.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectorGeneration::TrajectorGeneration(Node::SharedPtr nodeHandle,
                                         string config_yaml)
    : nodeHandle_(nodeHandle) {
  auto config_ = YAML::LoadFile(config_yaml);

  std::string robot_name = config_["model"]["name"].as<std::string>();
  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "model file: %s", urdf.c_str());

  freq_ = config_["generation"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "[TrajectorGeneration] frequency: %f",
              freq_);

  std::string ocs2_leg_robot_package =
      config_["ocs2"]["package"].as<std::string>();
  std::string reference_file =
      ament_index_cpp::get_package_share_directory(ocs2_leg_robot_package) +
      config_["ocs2"]["reference_file"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "reference file: %s",
              reference_file.c_str());

  std::string task_file =
      ament_index_cpp::get_package_share_directory(ocs2_leg_robot_package) +
      config_["ocs2"]["task_file"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "task file: %s", task_file.c_str());

  robot_interface_ptr_ =
      std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(
          task_file, urdf, reference_file);
  gait_receiver_ptr_ = std::make_shared<ocs2::legged_robot::GaitReceiver>(
      nodeHandle_,
      robot_interface_ptr_->getSwitchedModelReferenceManagerPtr()
          ->getGaitSchedule(),
      robot_name);
  reference_manager_ptr_ = std::make_shared<ocs2::RosReferenceManager>(
      robot_name, robot_interface_ptr_->getReferenceManagerPtr());
  reference_manager_ptr_->subscribe(nodeHandle_);

  conversions_ptr_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(
      robot_interface_ptr_->getPinocchioInterface(),
      robot_interface_ptr_->getCentroidalModelInfo());

  mpc_ptr_ = std::make_shared<ocs2::SqpMpc>(
      robot_interface_ptr_->mpcSettings(), robot_interface_ptr_->sqpSettings(),
      robot_interface_ptr_->getOptimalControlProblem(),
      robot_interface_ptr_->getInitializer());
  mpc_ptr_->getSolverPtr()->setReferenceManager(reference_manager_ptr_);
  mpc_ptr_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr_);
  mpc_ptr_->reset();

  vel_cmd.setZero();
  yawd_ = 0.0;

  run_.push(true);
  inner_loop_thread_ = std::thread(&TrajectorGeneration::inner_loop, this);
}

TrajectorGeneration::~TrajectorGeneration() {
  run_.push(false);
  inner_loop_thread_.join();
}

void TrajectorGeneration::update_current_state(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectorGeneration::set_reference() {
  if (mpc_sol_buffer.get() == nullptr) {
    ocs2::SystemObservation currentObservation;
    const vector_t rbdState = get_rbd_state();
    const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
    currentObservation.time = nodeHandle_->now().seconds();
    currentObservation.state =
        conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
    currentObservation.input = vector_t::Zero(info_.inputDim);
    currentObservation.mode = 15; // stance
    ocs2::TargetTrajectories initTargetTrajectories({currentObservation.time},
                                                    {currentObservation.state},
                                                    {currentObservation.input});

    mpc_ptr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(
        std::move(initTargetTrajectories));
  } else {
    ocs2::SystemObservation node1, node2;
    const vector_t rbdState = get_rbd_state();
    const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
    node1.time = nodeHandle_->now().seconds();
    node1.state =
        conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
    node1.state.head(6).setZero();
    node1.state.head(3) = vel_cmd;
    node1.state[3] = yawd_;
    if (0.3 - node1.state[8] > 0.03) {
      node1.state[8] += (0.02 * 0.5); // height
    } else {
      node1.state[8] = 0.3;
    }
    node1.state.segment(10, 2).setZero();
    node1.input = vector_t::Zero(info_.inputDim);
    node1.state.tail(12) << -0.1, 0.72, -1.46, 0.1, 0.72, -1.46, -0.1, 0.72,
        -1.48, 0.1, 0.72, -1.48;
    node1.mode = 15; // stance

    node2.time =
        nodeHandle_->now().seconds() + mpc_ptr_->settings().timeHorizon_ + 0.1;
    node2.state = node1.state;
    node2.state.segment(6, 3) +=
        (mpc_ptr_->settings().timeHorizon_ + 0.1) * vel_cmd;
    node2.state[9] += yawd_ * (mpc_ptr_->settings().timeHorizon_ + 0.1);
    node2.input = vector_t::Zero(info_.inputDim);
    node2.state.tail(12) << -0.1, 0.72, -1.46, 0.1, 0.72, -1.46, -0.1, 0.72,
        -1.48, 0.1, 0.72, -1.48;
    node2.mode = 15; // stance

    ocs2::TargetTrajectories initTargetTrajectories({node1.time, node2.time},
                                                    {node1.state, node2.state},
                                                    {node1.input, node2.input});
    mpc_ptr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(
        std::move(initTargetTrajectories));
  }
}

void TrajectorGeneration::inner_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();

    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr) {
      continue;

    } else {

      set_reference();

      ocs2::SystemObservation currentObservation;
      vector_t rbdState = get_rbd_state();
      currentObservation.time = nodeHandle_->now().seconds();
      currentObservation.state =
          conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
      bool mpcIsUpdated =
          mpc_ptr_->run(currentObservation.time, currentObservation.state);
      if (mpcIsUpdated) {
        auto primalSolutionPtr = std::make_unique<ocs2::PrimalSolution>();
        const scalar_t finalTime =
            (mpc_ptr_->settings().solutionTimeWindow_ < 0)
                ? mpc_ptr_->getSolverPtr()->getFinalTime()
                : currentObservation.time +
                      mpc_ptr_->settings().solutionTimeWindow_;
        mpc_ptr_->getSolverPtr()->getPrimalSolution(finalTime,
                                                    primalSolutionPtr.get());
        /* printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ "
               "$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
        for (size_t i = 0; i < primalSolutionPtr->timeTrajectory_.size(); i++) {
          std::cout
              << "sol t="
              << std::to_string(primalSolutionPtr->timeTrajectory_[i])
              << ", state: "
              << primalSolutionPtr->stateTrajectory_[i].head(12).transpose()
              << "\n"
              << "ref t="
              << std::to_string(primalSolutionPtr->timeTrajectory_[i])
              << ", state: "
              << reference_manager_ptr_->getTargetTrajectories()
                     .getDesiredState(primalSolutionPtr->timeTrajectory_[i])
                     .head(12)
                     .transpose()
              << "\n";
        } */
        mpc_sol_buffer.push(std::move(primalSolutionPtr));
      }
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(nodeHandle_->get_logger(),
              "[TrajectorGeneration] max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
  std::cerr << mpc_ptr_->getSolverPtr()->getBenchmarkingInfo();
}

void TrajectorGeneration::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  yawd_ = yawd;
}

vector_t TrajectorGeneration::get_rbd_state() {
  const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
  vector_t rbdState(2 * info_.generalizedCoordinatesNum);

  const auto qpos = *qpos_ptr_buffer.get();
  const auto qvel = *qvel_ptr_buffer.get();

  Eigen::Quaternion<scalar_t> quat(qpos[6], qpos[3], qpos[4], qpos[5]);
  vector_t b_angVel = qvel.segment(3, 3);
  vector_t w_angVel = quat.toRotationMatrix() * b_angVel;

  rbdState.head(3) = toEulerAnglesZYX(quat);
  rbdState.segment<3>(3) = qpos.head(3);
  rbdState.segment(info_.generalizedCoordinatesNum, 3) = w_angVel;
  rbdState.segment(info_.generalizedCoordinatesNum + 3, 3) =
      quat.toRotationMatrix() * qvel.head(3);
  assert(qpos.size() == (info_.actuatedDofNum + 6));
  rbdState.segment(6, info_.actuatedDofNum) = qpos.tail(info_.actuatedDofNum);
  rbdState.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) =
      qvel.tail(info_.actuatedDofNum);
  return rbdState;
}

std::shared_ptr<ocs2::PrimalSolution> TrajectorGeneration::get_mpc_sol() {
  return mpc_sol_buffer.get();
}

std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
TrajectorGeneration::get_robot_interface() {
  return robot_interface_ptr_;
}

} // namespace clear
