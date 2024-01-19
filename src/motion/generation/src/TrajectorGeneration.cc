#include "generation/TrajectorGeneration.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectorGeneration::TrajectorGeneration(Node::SharedPtr nodeHandle)
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

  mapping_ = std::make_shared<ocs2::CentroidalModelPinocchioMapping>(
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
  t0 = nodeHandle_->now().seconds();

  referenceBuffer_ = std::make_shared<ReferenceBuffer>();

  run_.push(true);
  inner_loop_thread_ = std::thread(&TrajectorGeneration::innerLoop, this);
}

TrajectorGeneration::~TrajectorGeneration() {
  run_.push(false);
  inner_loop_thread_.join();
}

std::shared_ptr<ReferenceBuffer> TrajectorGeneration::getReferenceBuffer() {
  return referenceBuffer_;
}

void TrajectorGeneration::updateCurrentState(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectorGeneration::setReference() {
  if (mpc_sol_buffer.get() == nullptr) {
    ocs2::SystemObservation currentObservation;
    const vector_t rbdState = getRbdState();

    const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
    currentObservation.time = nodeHandle_->now().seconds() - t0;
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
    const scalar_t t_now = nodeHandle_->now().seconds() - t0;
    ocs2::SystemObservation node1, node2;
    const vector_t rbdState = getRbdState();
    vector_t qpos = *(qpos_ptr_buffer.get());
    quaternion_t quat(qpos(6), qpos(3), qpos(4), qpos(5));
    vector3_t rpy_now = toEulerAngles(quat);
    rpy_now.head(2).setZero();
    vector3_t vel_cmd_world = toRotationMatrix(rpy_now) * vel_cmd;
    vel_cmd_world.z() = 0.0;

    /* scalar_t magn = 0;
    scalar_t alpha = 0.75;
    if (t_now > 6.0) {
      magn = 0.0 * (sin(alpha * (t_now - t0)) > 0 ? 1.0 : -1.0);
    }
    vel_cmd_world = vector3_t(magn, 0, 0); */

    if (first_run) {
      first_run = false;
      pos_start = rbdState.segment(3, 3);
      pos_start.z() = 0.3;
      rpy_zyx_start = rbdState.head(3);
      rpy_zyx_start.tail(2).setZero();
    } else {
      if ((rbdState.segment(3, 3) - pos_start).norm() < 0.2) {
        pos_start += 1.0 / freq_ * vel_cmd_world;
      }
      pos_start.z() = 0.3;
      if (compute_euler_angle_err(rbdState.head(3), rpy_zyx_start).norm() <
          0.2) {
        rpy_zyx_start.x() += 1.0 / freq_ * yawd_;
      }
      rpy_zyx_start.tail(2).setZero();
    }

    scalar_array_t timeTrajectory;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;

    size_t N = mpc_ptr_->getTimeHorizon() / 0.1;

    vector_t state =
        conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);
    vector3_t rpy_zyx_0 =
        state.segment(9, 3) -
        compute_euler_angle_err(state.segment(9, 3), rpy_zyx_start);
    for (size_t k = 0; k < N; k++) {
      timeTrajectory.push_back(t_now + 0.1 * k);
      vector_t state_des = vector_t::Zero(
          robot_interface_ptr_->getCentroidalModelInfo().stateDim);
      vector_t input_des = vector_t::Zero(
          robot_interface_ptr_->getCentroidalModelInfo().inputDim);

      state_des.head(6).setZero();
      state_des.head(3) = vel_cmd_world;
      state_des[5] = yawd_;
      state_des.segment(6, 3) = pos_start + 0.1 * k * vel_cmd_world;
      state_des[8] = 0.3;

      state_des.segment(9, 3) = rpy_zyx_0;
      state_des[9] = rpy_zyx_0.x() + yawd_ * k * 0.1;
      state_des.segment(10, 2).setZero();
      state_des.tail(12) << -0.2, 0.72, -1.46, 0.2, 0.72, -1.46, -0.2, 0.72,
          -1.48, 0.2, 0.72, -1.48;
      stateTrajectory.push_back(state_des);
      inputTrajectory.push_back(input_des);
    }

    ocs2::TargetTrajectories initTargetTrajectories(
        timeTrajectory, stateTrajectory, inputTrajectory);
    mpc_ptr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(
        std::move(initTargetTrajectories));

    std::vector<scalar_t> time;
    std::vector<vector_t> rpy_t, pos_t;
    N = mpc_ptr_->getTimeHorizon() / 0.05;
    for (size_t k = 0; k < N; k++) {
      time.push_back(t_now + 0.05 * k);
      vector3_t rpy_k;
      rpy_k.x() = rpy_zyx_0.z();
      rpy_k.y() = rpy_zyx_0.y();
      rpy_k.z() = rpy_zyx_0.x();
      // if (t_now - t0 > 2 * M_PI) {
      //   rpy_k.x() = 0.2 * sin(2 * (t_now - t0 + 0.05 * k));
      //   rpy_k.y() = 0.2 * sin(2 * (t_now - t0 + 0.05 * k));
      //   rpy_k.z() = 0.5 * sin(2 * (t_now - t0 + 0.05 * k));
      // }
      rpy_k.z() += 0.05 * k * yawd_;
      rpy_t.emplace_back(rpy_k);

      vector3_t pos_k = pos_start + 0.05 * k * vel_cmd_world;
      pos_k.z() = 0.3;
      pos_t.emplace_back(pos_k);
    }

    auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
    cubicspline_pos->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv, vel_cmd_world,
        CubicSplineInterpolation::BoundaryType::second_deriv,
        vector3_t::Zero());
    cubicspline_pos->fit(time, pos_t);
    referenceBuffer_->setIntegratedBasePosTraj(cubicspline_pos);

    auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
    cubicspline_rpy->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv,
        vector3_t(0, 0, yawd_),
        CubicSplineInterpolation::BoundaryType::second_deriv,
        vector3_t::Zero());
    cubicspline_rpy->fit(time, rpy_t);
    referenceBuffer_->setIntegratedBaseRpyTraj(cubicspline_rpy);
  }
}

void TrajectorGeneration::innerLoop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);

  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();

    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr) {
      continue;
    } else {
      setReference();

      ocs2::SystemObservation currentObservation;
      vector_t rbdState = getRbdState();
      currentObservation.time = nodeHandle_->now().seconds() - t0;
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
        mpc_sol_buffer.push(std::move(primalSolutionPtr));

        fitTraj();
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

vector_t TrajectorGeneration::getRbdState() {
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

void TrajectorGeneration::fitTraj() {
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
  std::vector<vector_t> rpy_t, pos_t, force_t;
  std::map<string, std::vector<vector_t>> foot_pos_array;
  std::map<string, vector3_t> foot_vel_array;

  const size_t n = mpc_sol->timeTrajectory_.size();
  for (size_t i = 1; i < n; i += 3) {
    time.push_back(mpc_sol->timeTrajectory_[i]);
    force_t.push_back(mpc_sol->inputTrajectory_[i - 1].head(12));
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
    vector3_t rpy_xyz(stateDesired(11), stateDesired(10), stateDesired(9));
    // rpy_t.push_back(toEulerAngles(base_pose.rotation()));
    rpy_t.push_back(rpy_xyz);
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
  referenceBuffer_->setOptimizedBasePosTraj(cubicspline_pos);

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
  referenceBuffer_->setOptimizedBaseRpyTraj(cubicspline_rpy);

  auto cubicspline_force = std::make_shared<CubicSplineTrajectory>(12);
  cubicspline_force->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv, vector_t::Zero(12),
      CubicSplineInterpolation::BoundaryType::first_deriv, vector_t::Zero(12));
  cubicspline_force->fit(time, force_t);
  referenceBuffer_->setOptimizedForceTraj(cubicspline_force);

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

std::shared_ptr<ocs2::PrimalSolution> TrajectorGeneration::getMpcSol() {
  return mpc_sol_buffer.get();
}

std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
TrajectorGeneration::getRobotInterface() {
  return robot_interface_ptr_;
}

} // namespace clear
