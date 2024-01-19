#include "control/WholeBodyController.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eiquadprog/eiquadprog-fast.hpp>
#include <pinocchio/Orientation.h>
#include <rcpputils/asserts.hpp>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace clear {

WholeBodyController::WholeBodyController(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
        robot_interface_ptr)
    : nodeHandle_(nodeHandle), robot_interface_ptr_(robot_interface_ptr) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "[WholeBodyController] model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

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

  conversions_ptr_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(
      robot_interface_ptr_->getPinocchioInterface(),
      robot_interface_ptr_->getCentroidalModelInfo());

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(nodeHandle_->get_logger(),
                "[WholeBodyController] foot name: %s", name.c_str());
  }
  pinocchioInterface_ptr_->setContactPoints(foot_names);

  base_name = config_["model"]["base_name"].as<std::string>();

  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();

  numDecisionVars_ = pinocchioInterface_ptr_->nv() + 3 * foot_names.size() +
                     actuated_joints_name.size();
  this->loadTasksSetting();

  t0 = nodeHandle_->now().seconds();
}

WholeBodyController::~WholeBodyController() {}

void WholeBodyController::updateReferenceBuffer(
    std::shared_ptr<ReferenceBuffer> referenceBuffer) {
  referenceBuffer_ = referenceBuffer;
}

void WholeBodyController::update_mpc_sol(
    std::shared_ptr<ocs2::PrimalSolution> mpc_sol) {
  mpc_sol_buffer.push(mpc_sol);
}

void WholeBodyController::update_state(
    const std::shared_ptr<vector_t> qpos_ptr,
    const std::shared_ptr<vector_t> qvel_ptr) {
  pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);
}

void WholeBodyController::updateBaseVectorField(
    const std::shared_ptr<VectorFieldParam> vf) {
  base_vf_.push(vf);
}

void WholeBodyController::formulate() {
  auto activePrimalSolutionPtr_ = mpc_sol_buffer.get();
  scalar_t currentTime = nodeHandle_->now().seconds() - t0;
  mode_ = activePrimalSolutionPtr_->modeSchedule_.modeAtTime(currentTime);

  ocs2::SystemObservation currentObservation;
  vector_t rbdState = get_rbd_state();
  currentObservation.time = nodeHandle_->now().seconds() - t0;
  currentObservation.state =
      conversions_ptr_->computeCentroidalStateFromRbdModel(rbdState);

  mpcInput = activePrimalSolutionPtr_->controllerPtr_->computeInput(
      currentTime, currentObservation.state);
  if (mpcInput_last.size() != mpcInput.size()) {
    mpcInput_last = mpcInput;
  }

  xDot = robot_interface_ptr_->getOptimalControlProblem()
             .dynamicsPtr->computeFlowMap(currentTime, currentObservation.state,
                                          mpcInput, ocs2::PreComputation());

  contactFlag_ = modeNumber2StanceLeg(mode_);
  numContacts_ = std::count(contactFlag_.cbegin(), contactFlag_.cend(), true);

  updateContactJacobi();

  constraints = formulateFloatingBaseEulerNewtonEqu() +
                formulateTorqueLimitsTask() + formulateFrictionConeTask() +
                formulateMaintainContactTask();
  weighedTask = formulateMomentumTask() + formulateSwingLegTask() +
                formulateContactForceTask();

  mpcInput_last = mpcInput;
}

std::shared_ptr<ActuatorCommands> WholeBodyController::optimize() {
  actuator_commands_ = std::make_shared<ActuatorCommands>();
  actuator_commands_->setZero(actuated_joints_name.size());
  if (mpc_sol_buffer.get().get() == nullptr) {
    return actuator_commands_;
  }

  formulate();

  matrix_t H = weighedTask.A.transpose() * weighedTask.A;
  H.diagonal() += 1e-12 * vector_t::Ones(numDecisionVars_);
  vector_t g = -weighedTask.A.transpose() * weighedTask.b;

  // Solve
  eiquadprog::solvers::EiquadprogFast eiquadprog_solver;
  eiquadprog_solver.reset(numDecisionVars_, constraints.b.size(),
                          2 * constraints.lb.size());
  matrix_t Cin(constraints.C.rows() * 2, numDecisionVars_);
  Cin << constraints.C, -constraints.C;
  vector_t cin(constraints.C.rows() * 2), ce0;
  cin << -constraints.lb, constraints.ub;
  ce0 = -constraints.b;
  vector_t optimal_u = vector_t::Zero(numDecisionVars_);
  auto solver_state = eiquadprog_solver.solve_quadprog(H, g, constraints.A, ce0,
                                                       Cin, cin, optimal_u);
  // printf("solver state: %d\n", solver_state);
  if (solver_state == eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
    actuator_commands_->torque = optimal_u.tail(actuated_joints_name.size());
    joint_acc_ = optimal_u.head(pinocchioInterface_ptr_->nv())
                     .tail(actuated_joints_name.size());
    differential_inv_kin();
  } else {
    joint_acc_.setZero(actuated_joints_name.size());
    std::cerr << "wbc failed ...\n";
    actuator_commands_->setZero(actuated_joints_name.size());
    actuator_commands_->Kd.fill(-1.0);
  }

  return actuator_commands_;
}

void WholeBodyController::updateContactJacobi() {
  Jc = matrix_t(3 * foot_names.size(), pinocchioInterface_ptr_->nv());
  for (size_t i = 0; i < foot_names.size(); ++i) {
    matrix6x_t jac;
    pinocchioInterface_ptr_->getJacobia_localWorldAligned(foot_names[i], jac);
    Jc.middleRows(3 * i, 3) = jac.topRows(3);
  }
}

MatrixDB WholeBodyController::formulateFloatingBaseEulerNewtonEqu() {
  MatrixDB eulerNewtonEqu("eulerNewtonEqu");
  auto &data = pinocchioInterface_ptr_->getData();
  size_t nv = pinocchioInterface_ptr_->nv();
  size_t na = actuated_joints_name.size();

  if (nv != na + 6) {
    throw std::runtime_error("nv != info_.actuatedDofNum + 6");
  }
  matrix_t S(nv, na);
  S.topRows(6).setZero();
  S.bottomRows(na).setIdentity();
  eulerNewtonEqu.A =
      (matrix_t(nv, numDecisionVars_) << data.M, -Jc.transpose(), -S)
          .finished();
  eulerNewtonEqu.b = -data.nle;
  return eulerNewtonEqu;
}

MatrixDB WholeBodyController::formulateTorqueLimitsTask() {
  MatrixDB limit_tau("limit_tau");
  size_t na = actuated_joints_name.size();
  limit_tau.C.setZero(na, numDecisionVars_);
  limit_tau.C.bottomRightCorner(na, na).setIdentity();
  limit_tau.lb = -pinocchioInterface_ptr_->getModel().effortLimit.tail(na);
  limit_tau.ub = pinocchioInterface_ptr_->getModel().effortLimit.tail(na);
  return limit_tau;
}

MatrixDB WholeBodyController::formulateMaintainContactTask() {
  MatrixDB contact_task("contact_task");
  size_t nc = foot_names.size();
  size_t nv = pinocchioInterface_ptr_->nv();
  contact_task.A.setZero(3 * numContacts_, numDecisionVars_);
  contact_task.b.setZero(3 * numContacts_);
  size_t j = 0;
  for (size_t i = 0; i < nc; i++) {
    if (contactFlag_[i]) {
      contact_task.A.block(3 * j, 0, 3, nv) = Jc.middleRows(3 * i, 3);
      contact_task.b.segment(3 * j, 3) =
          -pinocchioInterface_ptr_
               ->getFrame6dAcc_localWorldAligned(foot_names[i])
               .linear();
      j++;
    }
  }
  return contact_task;
}

MatrixDB WholeBodyController::formulateFrictionConeTask() {
  MatrixDB friction_cone("friction_cone");
  size_t nc = foot_names.size();
  size_t nv = pinocchioInterface_ptr_->nv();
  size_t j = 0;

  friction_cone.A.setZero(3 * (nc - numContacts_), numDecisionVars_);
  for (size_t i = 0; i < nc; ++i) {
    if (!contactFlag_[i]) {
      friction_cone.A.block(3 * j++, nv + 3 * i, 3, 3) =
          matrix_t::Identity(3, 3);
    }
  }
  friction_cone.b.setZero(friction_cone.A.rows());

  matrix_t frictionPyramic(5, 3); // clang-format off
  frictionPyramic << 0, 0, 1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_; // clang-format on
  friction_cone.C.setZero(5 * numContacts_, numDecisionVars_);
  friction_cone.ub = Eigen::VectorXd::Zero(friction_cone.C.rows());
  friction_cone.lb = -1e16 * Eigen::VectorXd::Ones(friction_cone.C.rows());

  j = 0;
  for (size_t i = 0; i < nc; ++i) {
    if (contactFlag_[i]) {
      friction_cone.C.block(5 * j, nv + 3 * i, 5, 3) = frictionPyramic;
      friction_cone.lb(5 * j) = 0.0;
      friction_cone.ub(5 * j) = 400.0;
      j++;
    }
  }
  return friction_cone;
}

MatrixDB WholeBodyController::formulateMomentumTask() {
  size_t nv = pinocchioInterface_ptr_->nv();
  const auto policy = base_vf_.get();
  if (policy != nullptr) {
    MatrixDB base_task("base_task");
    base_task.A.setZero(6, numDecisionVars_);
    matrix6x_t J = matrix6x_t::Zero(6, nv);
    pinocchioInterface_ptr_->getJacobia_local(base_name, J);
    base_task.A.leftCols(nv) = J;

    vector_t x0(12);
    auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
    auto base_twist =
        pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
    vector_t rpy = toEulerAngles(base_pose.rotation());

    x0 << base_pose.translation(), base_twist.linear(), rpy,
        base_twist.angular();
    vector6_t acc_fb = policy->K * x0 + policy->b;
    // to local coordinate
    acc_fb.head(3) = base_pose.rotation().transpose() * acc_fb.head(3);
    acc_fb.tail(3) = base_pose.rotation().transpose() * acc_fb.tail(3);

    base_task.b =
        acc_fb -
        pinocchioInterface_ptr_->getFrame6dAcc_local(base_name).toVector();
    // std::cout << "acc_fb: " << acc_fb.transpose() << "\n";

    base_task.A = weightMomentum_ * base_task.A;
    base_task.b = weightMomentum_ * base_task.b;
    return base_task;
  } else {
    MatrixDB momentum_task("momentum_task");

    momentum_task.A.setZero(6, numDecisionVars_);
    momentum_task.b.setZero(6);

    momentum_task.A.topLeftCorner(6, nv) =
        pinocchioInterface_ptr_->getMomentumJacobia();

    momentum_task.b = pinocchioInterface_ptr_->total_mass() * xDot.head(6) -
                      pinocchioInterface_ptr_->getMomentumTimeVariation();

    momentum_task.A = weightMomentum_ * momentum_task.A;
    momentum_task.b = weightMomentum_ * momentum_task.b;

    return momentum_task;
  }
}

MatrixDB WholeBodyController::formulateSwingLegTask() {
  const size_t nc = foot_names.size();
  const size_t nv = pinocchioInterface_ptr_->nv();

  auto foot_traj = referenceBuffer_.get()->getFootPosTraj();
  auto base_pos_traj = referenceBuffer_->getIntegratedBasePosTraj();

  if (nc - numContacts_ <= 0 || foot_traj.size() != nc ||
      base_pos_traj.get() == nullptr) {
    return MatrixDB("swing_task");
  }
  MatrixDB swing_task("swing_task");
  scalar_t t = nodeHandle_->now().seconds() - t0 + dt_;

  matrix_t Qw =
      matrix_t::Zero(3 * (nc - numContacts_), 3 * (nc - numContacts_));
  swing_task.A.setZero(3 * (nc - numContacts_), numDecisionVars_);
  swing_task.b.setZero(swing_task.A.rows());

  size_t j = 0;
  for (size_t i = 0; i < nc; ++i) {
    const auto &foot_name = foot_names[i];
    if (!contactFlag_[i]) {
      Qw.block<3, 3>(3 * j, 3 * j) = weightSwingLeg_;
      const auto traj = foot_traj[foot_name];
      vector3_t pos_des =
          traj->evaluate(t) - base_pos_traj->evaluate(t) +
          pinocchioInterface_ptr_->getFramePose(base_name).translation();
      vector3_t vel_des =
          traj->derivative(t, 1) - base_pos_traj->derivative(t, 1) +
          pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name)
              .linear();
      vector3_t acc_des = traj->derivative(t, 2);
      vector3_t pos_m =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      vector3_t vel_m =
          pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(foot_name)
              .linear();
      vector3_t pos_err = pos_des - pos_m;
      vector3_t vel_err = vel_des - vel_m;
      vector3_t accel_fb = swingKp_ * pos_err + swingKd_ * vel_err;
      /* if (accel_fb.norm() > 10.0) {
        accel_fb = 10.0 * accel_fb.normalized();
      } */
      swing_task.A.block(3 * j, 0, 3, nv) = Jc.block(3 * i, 0, 3, nv);
      swing_task.b.segment(3 * j, 3) =
          -pinocchioInterface_ptr_->getFrame6dAcc_localWorldAligned(foot_name)
               .linear() +
          accel_fb + acc_des;
      j++;
    }
  }

  swing_task.A.leftCols(6).setZero();
  swing_task.A = Qw * swing_task.A;
  swing_task.b = Qw * swing_task.b;
  return swing_task;
}

void WholeBodyController::differential_inv_kin() {
  auto activePrimalSolutionPtr_ = mpc_sol_buffer.get();

  vector_t joints_pos_des = ocs2::LinearInterpolation::interpolate(
                                nodeHandle_->now().seconds() - t0,
                                activePrimalSolutionPtr_->timeTrajectory_,
                                activePrimalSolutionPtr_->stateTrajectory_)
                                .tail(actuated_joints_name.size());

  vector_t joints_vel_des = ocs2::LinearInterpolation::interpolate(
                                nodeHandle_->now().seconds() - t0,
                                activePrimalSolutionPtr_->timeTrajectory_,
                                activePrimalSolutionPtr_->inputTrajectory_)
                                .tail(actuated_joints_name.size());

  for (size_t i = 0; i < actuated_joints_name.size(); i++) {
    actuator_commands_->Kp(i) = 30.0;
    actuator_commands_->Kd(i) = 1.0;
    actuator_commands_->pos(i) = joints_pos_des(i);
    actuator_commands_->vel(i) = joints_vel_des(i);
  }
}

MatrixDB WholeBodyController::formulateContactForceTask() {
  // size_t nc = foot_names.size();
  // size_t nv = pinocchioInterface_ptr_->nv();

  // MatrixDB contact_force("contact_force");
  // contact_force.A.setZero(3 * nc, numDecisionVars_);
  // contact_force.A.middleCols(nv, 3 * nc).setIdentity();

  // contact_force.b = mpcInput.head(3 * nc);

  // contact_force.A = weightContactForce_ * contact_force.A;
  // contact_force.b = weightContactForce_ * contact_force.b;
  // return contact_force;

  size_t nc = foot_names.size();
  size_t nv = pinocchioInterface_ptr_->nv();
  const auto policy = base_vf_.get();

  MatrixDB contact_force("contact_force");
  contact_force.A.setZero(3 * nc, numDecisionVars_);
  if (policy != nullptr) {
    contact_force.b = policy->force_des;
    weightContactForce_ = 5.0;
  } else {
    contact_force.b = mpcInput.head(3 * nc);
    weightContactForce_ = 5.0;
  }
  for (size_t i = 0; i < nc; ++i) {
    contact_force.A.block<3, 3>(3 * i, nv + 3 * i) = matrix_t::Identity(3, 3);
  }
  contact_force.A = weightContactForce_ * contact_force.A;
  contact_force.b = weightContactForce_ * contact_force.b;
  return contact_force;
}

void WholeBodyController::loadTasksSetting(bool verbose) {
  // Load task file
  weightMomentum_.setZero(6, 6);
  weightMomentum_.diagonal().fill(100);

  weightSwingLeg_.setZero(3, 3);
  weightSwingLeg_.diagonal().fill(200);

  weightContactForce_ = 1e-2;

  frictionCoeff_ = 0.5;

  swingKp_.setZero(3, 3);
  swingKp_.diagonal().fill(350);

  swingKd_.setZero(3, 3);
  swingKd_.diagonal().fill(37);

  if (verbose) {
    std::cerr << "\n ########### weights.momentum ########### \n";
    std::cerr << weightMomentum_ << "\n";
    std::cerr << "\n ########### weights.floatingbase ########### \n";
    std::cerr << "\n ########### weights.leg_swing ########### \n";
    std::cerr << weightSwingLeg_ << "\n";
    std::cerr << "\n ########### weights.weightContactForce_: "
              << weightContactForce_ << "\n";
    std::cerr << "\n ########### friction_coefficient: " << frictionCoeff_
              << "\n";

    std::cerr << "\n ########### feedback_gain.leg_swing.kp ########### \n";
    std::cerr << swingKp_ << "\n";
    std::cerr << "\n ########### feedback_gain.leg_swing.kd ########### \n";
    std::cerr << swingKd_ << "\n";
    std::cerr << "\n ########### feedback_gain.floatingbase.kp ########### \n";
  }
}

vector_t WholeBodyController::get_rbd_state() {
  auto activePrimalSolutionPtr_ = mpc_sol_buffer.get();
  const auto info_ = robot_interface_ptr_->getCentroidalModelInfo();
  vector_t rbdState(2 * info_.generalizedCoordinatesNum);

  const auto qpos = pinocchioInterface_ptr_->qpos();
  const auto qvel = pinocchioInterface_ptr_->qvel();

  Eigen::Quaternion<scalar_t> quat(qpos[6], qpos[3], qpos[4], qpos[5]);
  vector_t b_angVel = qvel.segment(3, 3);
  vector_t w_angVel = quat.toRotationMatrix() * b_angVel;

  vector_t euler_zyx = toEulerAnglesZYX(quat);
  vector_t euler_zyx_des = ocs2::LinearInterpolation::interpolate(
                               nodeHandle_->now().seconds() - t0,
                               activePrimalSolutionPtr_->timeTrajectory_,
                               activePrimalSolutionPtr_->stateTrajectory_)
                               .segment(9, 3);

  for (size_t i = 0; i < 3; i++) {
    if (abs(euler_zyx(i) - euler_zyx_des(i)) > M_PI) {
      if (euler_zyx(i) < euler_zyx_des(i)) {
        euler_zyx(i) += 2.0 * M_PI;
      } else {
        euler_zyx(i) -= 2.0 * M_PI;
      }
    }
  }

  rbdState.head(3) = euler_zyx;
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

} // namespace clear