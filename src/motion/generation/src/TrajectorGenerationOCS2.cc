#include "generation/TrajectorGenerationOCS2.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectorGenerationOCS2::TrajectorGenerationOCS2(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);

  std::string robot_name = config_["model"]["name"].as<std::string>();
  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "model file: %s", urdf.c_str());

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();

  base_name = config_["model"]["base_name"].as<std::string>();

  freq_ = config_["generation"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(nodeHandle_->get_logger(),
              "[TrajectorGenerationOCS2] frequency: %f", freq_);

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

  conversions_ptr_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(
      robot_interface_ptr_->getPinocchioInterface(),
      robot_interface_ptr_->getCentroidalModelInfo());

  mapping_ = std::make_shared<ocs2::CentroidalModelPinocchioMapping>(
      robot_interface_ptr_->getCentroidalModelInfo());

  mrt_ptr_ = std::make_shared<ocs2::MRT_ROS_Interface>(nodeHandle_, robot_name);
  mrt_ptr_->initRollout(&robot_interface_ptr_->getRollout());

  ocs2::CentroidalModelPinocchioMapping pinocchioMapping(
      robot_interface_ptr_->getCentroidalModelInfo());
  ocs2::PinocchioEndEffectorKinematics endEffectorKinematics(
      robot_interface_ptr_->getPinocchioInterface(), pinocchioMapping,
      robot_interface_ptr_->modelSettings().contactNames3DoF);
  visualizer_ptr_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(
      robot_interface_ptr_->getPinocchioInterface(),
      robot_interface_ptr_->getCentroidalModelInfo(), endEffectorKinematics,
      nodeHandle_);

  mrtDesiredFrequency_ =
      robot_interface_ptr_->mpcSettings().mrtDesiredFrequency_;
  mpcDesiredFrequency_ =
      robot_interface_ptr_->mpcSettings().mpcDesiredFrequency_;

  vel_cmd.setZero();
  yawd_ = 0.0;

  referenceBuffer_ = std::make_shared<ReferenceBuffer>();

  mpcInit();

  run_.push(true);
  inner_loop_thread_ = std::thread(&TrajectorGenerationOCS2::innerLoop, this);
}

TrajectorGenerationOCS2::~TrajectorGenerationOCS2() {
  run_.push(false);
  inner_loop_thread_.join();
}

std::shared_ptr<ReferenceBuffer> TrajectorGenerationOCS2::getReferenceBuffer() {
  return referenceBuffer_;
}

void TrajectorGenerationOCS2::updateCurrentState(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectorGenerationOCS2::mpcInit() {
  RCLCPP_WARN_STREAM(nodeHandle_->get_logger(),
                     "Waiting for the initial policy ...");

  ocs2::SystemObservation initObservation;
  initObservation.state = robot_interface_ptr_->getInitialState();
  initObservation.input =
      vector_t::Zero(robot_interface_ptr_->getCentroidalModelInfo().inputDim);
  initObservation.mode = ocs2::legged_robot::ModeNumber::STANCE;

  // Initial command
  ocs2::TargetTrajectories initTargetTrajectories(
      {0.0}, {initObservation.state}, {initObservation.input});
  // Reset MPC node
  mrt_ptr_->resetMpc(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt_ptr_->initialPolicyReceived() && rclcpp::ok()) {
    rclcpp::spin_some(nodeHandle_);
    mrt_ptr_->setCurrentObservation(initObservation);
    rclcpp::Rate(robot_interface_ptr_->mpcSettings().mrtDesiredFrequency_)
        .sleep();
  }
  RCLCPP_INFO_STREAM(nodeHandle_->get_logger(),
                     "Initial policy has been received.");
}

void TrajectorGenerationOCS2::setReference() {
  if (mpc_sol_buffer.get() == nullptr) {
    ocs2::SystemObservation currentObservation;
    const vector_t rbdState = getRbdState();

    const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
    currentObservation.time = nodeHandle_->now().seconds();
    currentObservation.state =
        conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
    currentObservation.input = vector_t::Zero(info_.inputDim);
    currentObservation.mode = 15; // stance
    ocs2::TargetTrajectories initTargetTrajectories({currentObservation.time},
                                                    {currentObservation.state},
                                                    {currentObservation.input});

  } else {
    ocs2::SystemObservation node1, node2;
    const vector_t rbdState = getRbdState();

    if (first_run) {
      first_run = false;
      pos_start = rbdState.segment(3, 3);
      pos_start.z() = 0.3;
      rpy_zyx_start = rbdState.head(3);
      rpy_zyx_start.tail(2).setZero();
    } else {
      vector_t qpos = *(qpos_ptr_buffer.get());
      quaternion_t quat(qpos(6), qpos(3), qpos(4), qpos(5));
      pos_start += 1.0 / freq_ * quat.toRotationMatrix() * vel_cmd;
      pos_start.z() = 0.3;
      rpy_zyx_start.x() += 1.0 / freq_ * yawd_;
      rpy_zyx_start.tail(2).setZero();
    }

    const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
    node1.time = nodeHandle_->now().seconds();
    node1.state =
        conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
    node1.state.head(6).setZero();
    node1.state.head(3) = vel_cmd;
    node1.state[3] = yawd_;
    node1.state.segment(6, 3) = pos_start;
    if (0.3 - node1.state[8] > 0.03) {
      node1.state[8] += (0.02 * 0.5); // height
    } else {
      node1.state[8] = 0.3;
    }

    node1.state.segment(9, 3) = rpy_zyx_start;
    node1.state.segment(10, 2).setZero();
    node1.input = vector_t::Zero(info_.inputDim);
    node1.state.tail(12) << -0.1, 0.72, -1.46, 0.1, 0.72, -1.46, -0.1, 0.72,
        -1.48, 0.1, 0.72, -1.48;
    node1.mode = 15; // stance

    node2.time = nodeHandle_->now().seconds() + 1.1;
    node2.state = node1.state;
    node2.state.segment(6, 3) += 1.1 * vel_cmd;
    node2.state[9] += yawd_ * 1.1;
    node2.input = vector_t::Zero(info_.inputDim);
    node2.state.tail(12) << -0.1, 0.72, -1.46, 0.1, 0.72, -1.46, -0.1, 0.72,
        -1.48, 0.1, 0.72, -1.48;
    node2.mode = 15; // stance

    ocs2::TargetTrajectories initTargetTrajectories({node1.time, node2.time},
                                                    {node1.state, node2.state},
                                                    {node1.input, node2.input});
  }
}

void TrajectorGenerationOCS2::innerLoop() {
  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an
  // old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) {
    constexpr scalar_t tol =
        0.1; // policy must start within this fraction of dt
    return mrt_ptr_->updatePolicy() &&
           std::abs(mrt_ptr_->getPolicy().timeTrajectory_.front() - time) <
               (tol / mpcDesiredFrequency_);
  };

  rclcpp::Rate loop_rate(mpcDesiredFrequency_);
  rclcpp::Rate loop_mrt_rate(mrtDesiredFrequency_);
  benchmark::RepeatedTimer timer_;
  const scalar_t t0 = nodeHandle_->now().seconds();
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGenerationOCS2"), "t0: %f", t0);

  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();

    if (qpos_ptr_buffer.get().get() != nullptr &&
        qvel_ptr_buffer.get().get() != nullptr) {

      const scalar_t t = nodeHandle_->now().seconds() - t0;

      /* RCLCPP_INFO(nodeHandle_->get_logger(),
                  "[TrajectorGeneration] Current time: %f", t); */

      ocs2::SystemObservation currentObservation;
      vector_t rbdState = getRbdState();
      currentObservation.time = t;
      currentObservation.state =
          conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
      currentObservation.mode = ocs2::legged_robot::ModeNumber::STANCE;
      currentObservation.input = vector_t::Zero(
          robot_interface_ptr_->getCentroidalModelInfo().inputDim);

      mrt_ptr_->setCurrentObservation(currentObservation);

      /* RCLCPP_INFO(nodeHandle_->get_logger(),
                  "[TrajectorGeneration] Observation is published at: %f", t);
       */

      while (!policyUpdatedForTime(t) && rclcpp::ok()) {
        loop_mrt_rate.sleep();
        // RCLCPP_INFO(nodeHandle_->get_logger(),
        //             "[TrajectorGeneration] Wait for the policy to be
        //             updated");
      }

      auto mpc_sol = std::make_shared<ocs2::PrimalSolution>();
      *mpc_sol = mrt_ptr_->getPolicy();
      mpc_sol_buffer.push(mpc_sol);
      // fitTraj();
      /* RCLCPP_INFO(nodeHandle_->get_logger(),
                  "[TrajectorGeneration] New MPC policy starting at: %f",
                  mrt_ptr_->getPolicy().timeTrajectory_.front()); */
      // Update vis
      visualizer_ptr_->update(currentObservation, mrt_ptr_->getPolicy(),
                              mrt_ptr_->getCommand());

      timer_.endTimer();
      loop_rate.sleep();
    }
  }

  RCLCPP_INFO(nodeHandle_->get_logger(),
              "[TrajectorGeneration] max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void TrajectorGenerationOCS2::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  yawd_ = yawd;
}

vector_t TrajectorGenerationOCS2::getRbdState() {
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

void TrajectorGenerationOCS2::fitTraj() {
  auto mpc_sol = mpc_sol_buffer.get();
  if (mpc_sol.get() == nullptr) {
    return;
  }

  ocs2::PinocchioInterface pinocchioInterface =
      robot_interface_ptr_->getPinocchioInterface();
  const auto &model = pinocchioInterface.getModel();
  auto &data = pinocchioInterface.getData();

  std::vector<scalar_t> time;
  vector3_t vel_start;
  std::vector<vector_t> rpy_t, pos_t;
  std::map<string, std::vector<vector_t>> foot_pos_array;
  std::map<string, vector3_t> foot_vel_array;

  const size_t n = mpc_sol->timeTrajectory_.size();
  for (size_t i = 1; i < n; i += 3) {
    time.push_back(mpc_sol->timeTrajectory_[i]);
    vector_t stateDesired = mpc_sol->stateTrajectory_[i];
    vector_t inputDesired = mpc_sol->inputTrajectory_[i];
    mapping_->setPinocchioInterface(pinocchioInterface);
    const auto qDesired = mapping_->getPinocchioJointPosition(stateDesired);
    pinocchio::forwardKinematics(model, data, qDesired);
    pinocchio::updateFramePlacements(model, data);
    ocs2::updateCentroidalDynamics(
        pinocchioInterface, robot_interface_ptr_->getCentroidalModelInfo(),
        qDesired);
    const vector_t vDesired =
        mapping_->getPinocchioJointVelocity(stateDesired, inputDesired);
    pinocchio::forwardKinematics(model, data, qDesired, vDesired);

    auto base_pose = data.oMf[model.getFrameId(base_name)];
    rpy_t.push_back(toEulerAngles(base_pose.rotation()));
    pos_t.push_back(base_pose.translation());

    if (i == 1) {
      auto base_twist =
          pinocchio::getFrameVelocity(model, data, model.getFrameId(base_name),
                                      pinocchio::LOCAL_WORLD_ALIGNED);
      vel_start = base_twist.linear();
    }

    for (auto name : foot_names) {
      foot_pos_array[name].push_back(
          data.oMf[model.getFrameId(name)].translation());
      if (i == 1) {
        auto twist =
            pinocchio::getFrameVelocity(model, data, model.getFrameId(name),
                                        pinocchio::LOCAL_WORLD_ALIGNED);
        foot_vel_array[name] = twist.linear();
      }
    }
  }

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv, vel_start,
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_t);
  referenceBuffer_->setIntegratedBasePosTraj(cubicspline_pos);

  /* for (size_t i = 0; i < time.size(); i++)
  {
    std::cout << " t: " << time[i] - time.front()
              << " pos: " << pos_t[i].transpose() << " " <<
  cubicspline_pos->evaluate(time[i]).transpose() << "\n";
  } */

  auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_rpy->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector3_t(0, 0, yawd_),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_rpy->fit(time, rpy_t);
  referenceBuffer_->setIntegratedBaseRpyTraj(cubicspline_rpy);

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>> foot_pos_traj;
  for (auto name : foot_names) {
    auto cubicspline_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline);
    cubicspline_->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv,
        foot_vel_array[name],
        CubicSplineInterpolation::BoundaryType::second_deriv,
        vector3_t::Zero());
    cubicspline_->fit(time, foot_pos_array[name]);
    foot_pos_traj[name] = std::move(cubicspline_);
  }
  referenceBuffer_->setFootPosTraj(foot_pos_traj);
}

std::shared_ptr<ocs2::PrimalSolution> TrajectorGenerationOCS2::getMpcSol() {
  return mpc_sol_buffer.get();
}

std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
TrajectorGenerationOCS2::getRobotInterface() {
  return robot_interface_ptr_;
}

} // namespace clear
