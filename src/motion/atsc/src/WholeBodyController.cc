#include "atsc/WholeBodyController.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/optimization/MathematicalProgram.h>
#include <pinocchio/Orientation.h>
#include <rcpputils/asserts.hpp>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace clear {

WholeBodyController::WholeBodyController(Node::SharedPtr nodeHandle,
                                         const std::string config_yaml)
    : nodeHandle_(nodeHandle),
      log_stream("log_stream_wbc.txt", std::ios::ate | std::ios::out) {
  auto config_ = YAML::LoadFile(config_yaml);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "[WholeBodyController] model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

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
}

WholeBodyController::~WholeBodyController() { log_stream.close(); }

void WholeBodyController::update_trajectory_reference(
    std::shared_ptr<TrajectoriesArray> referenceTrajectoriesPtr) {
  refTrajBuffer_.push(referenceTrajectoriesPtr);
}

void WholeBodyController::update_mode(size_t mode) { mode_.push(mode); }

void WholeBodyController::update_state(
    const std::shared_ptr<vector_t> qpos_ptr,
    const std::shared_ptr<vector_t> qvel_ptr) {
  pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);
}

void WholeBodyController::update_base_policy(
    const std::shared_ptr<AdaptiveGain::FeedbackGain> policy) {
  base_policy_.push(policy);
}

void WholeBodyController::update_swing_policy(
    const std::shared_ptr<AdaptiveGain::FeedbackGain> policy) {
  swing_policy_.push(policy);
}

void WholeBodyController::formulate() {
  contactFlag_ = modeNumber2StanceLeg(mode_.get());
  numContacts_ = std::count(contactFlag_.cbegin(), contactFlag_.cend(), true);

  updateContactJacobi();

  constraints = formulateFloatingBaseEulerNewtonEqu() +
                formulateTorqueLimitsTask() + formulateFrictionConeTask() +
                formulateMaintainContactTask();
  weighedTask = formulateBaseTask() + formulateSwingLegTask() +
                formulateContactForceTask();
}

std::shared_ptr<ActuatorCommands> WholeBodyController::optimize() {
  actuator_commands_ = std::make_shared<ActuatorCommands>();
  actuator_commands_->setZero(actuated_joints_name.size());
  if (base_policy_.get().get() == nullptr) {
    return actuator_commands_;
  }

  formulate();

  matrix_t H = weighedTask.A.transpose() * weighedTask.A;
  // H.diagonal() += 1e-8 * vector_t::Ones(numDecisionVars_);
  vector_t g = -weighedTask.A.transpose() * weighedTask.b;

  // Solve
  MathematicalProgram prog;
  auto var = prog.newVectorVariables(numDecisionVars_);
  prog.addLinearEqualityConstraints(constraints.A, constraints.b, var);
  prog.addLinearInEqualityConstraints(constraints.C, constraints.lb,
                                      constraints.ub, var);
  prog.addQuadraticCost(H, g, var);
  vector_t tau;
  if (prog.solve()) {
    actuator_commands_->torque =
        prog.getSolution(var).tail(actuated_joints_name.size());
    joint_acc_ = prog.getSolution()
                     .head(pinocchioInterface_ptr_->nv())
                     .tail(actuated_joints_name.size());
  } else {
    joint_acc_.setZero(actuated_joints_name.size());
    std::cerr << "wbc failed ...\n";
    actuator_commands_->torque =
        pinocchioInterface_ptr_->nle().tail(actuated_joints_name.size());
  }
  differential_inv_kin();

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

MatrixDB WholeBodyController::formulateBaseTask() {
  MatrixDB base_task("base_task");
  size_t nv = pinocchioInterface_ptr_->nv();
  const auto policy = base_policy_.get();

  base_task.A.setZero(6, numDecisionVars_);
  matrix6x_t J = matrix6x_t::Zero(6, nv);
  pinocchioInterface_ptr_->getJacobia_local(base_name, J);
  base_task.A.leftCols(nv) = J;

  vector6_t acc_fb;
  auto pos_traj = refTrajBuffer_.get()->get_base_pos_traj();
  auto rpy_traj = refTrajBuffer_.get()->get_base_rpy_traj();

  auto err_pos_traj = refTrajBuffer_.get()->get_optimized_base_pos_traj();
  auto err_rpy_traj = refTrajBuffer_.get()->get_optimized_base_rpy_traj();

  if (policy.get() != nullptr && pos_traj.get() != nullptr &&
      rpy_traj.get() != nullptr) {
    scalar_t t = nodeHandle_->now().seconds() + dt_;
    vector_t x0(12);
    auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
    auto base_twist =
        pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
    vector_t rpy = toEulerAngles(base_pose.rotation());
    vector3_t rpy_err = compute_euler_angle_err(rpy, rpy_traj->evaluate(t));
    vector3_t omega_des =
        getJacobiFromRPYToOmega(rpy) * rpy_traj->derivative(t, 1);

    x0 << base_pose.translation() - pos_traj->evaluate(t),
        base_twist.linear() - pos_traj->derivative(t, 1), rpy_err,
        pinocchioInterface_ptr_->getData().Ig.inertia() *
            (base_twist.angular() - omega_des);
    acc_fb = policy->K * x0 + policy->b;
    if (acc_fb.norm() > 20) {
      acc_fb = 20.0 * acc_fb.normalized();
    }
    // to local coordinate
    acc_fb.head(3) = base_pose.rotation().transpose() * acc_fb.head(3);
    acc_fb.tail(3) = base_pose.rotation().transpose() * acc_fb.tail(3);
    log_stream
        << (err_pos_traj->evaluate(t) - x0.head(3)).transpose() << " "
        << (err_rpy_traj->evaluate(t) - rpy_err).transpose() << " "
        << (err_pos_traj->derivative(t, 1) - x0.segment(3, 3)).transpose()
        << " "
        << (getJacobiFromRPYToOmega(rpy) * err_rpy_traj->derivative(t, 1) -
            (base_twist.angular() - omega_des))
               .transpose()
        << " " << acc_fb.transpose() << "\n";
  } else {
    acc_fb.setZero();
  }

  base_task.b =
      acc_fb -
      pinocchioInterface_ptr_->getFrame6dAcc_local(base_name).toVector();
  // std::cout << "acc_fb: " << acc_fb.transpose() << "\n";

  base_task.A = weightBase_ * base_task.A;
  base_task.b = weightBase_ * base_task.b;

  return base_task;
}

MatrixDB WholeBodyController::formulateSwingLegTask() {
  const size_t nc = foot_names.size();
  const size_t nv = pinocchioInterface_ptr_->nv();
  const size_t nj = actuated_joints_name.size();

  auto foot_traj = refTrajBuffer_.get()->get_foot_pos_traj();
  auto jnt_pos_traj = refTrajBuffer_.get()->get_joint_pos_traj();

  if (nc - numContacts_ <= 0 || foot_traj.size() != nc) {
    return MatrixDB("swing_task");
  }
  MatrixDB swing_task("swing_task");
  scalar_t t = nodeHandle_->now().seconds() + dt_;

  if (false) {
    auto policy = swing_policy_.get();
    vector_t err_jnt(2 * nj);
    err_jnt << pinocchioInterface_ptr_->qpos().tail(nj) -
                   jnt_pos_traj->evaluate(t),
        pinocchioInterface_ptr_->qvel().tail(nj) -
            jnt_pos_traj->derivative(t, 1);
    vector_t tau_des = policy->K * err_jnt + policy->b;
    swing_task.A.setZero(nj, numDecisionVars_);
    swing_task.A.rightCols(nj).setIdentity();
    swing_task.b = tau_des;

    /* swing_task.A = 1e2 * swing_task.A;
    swing_task.b = 1e2 * swing_task.b; */

    /* std::cout << "des jnt_acc: " << jnt_pos_traj->derivative(t,
    2).transpose()
              << "\n";
    std::cout << "fb jnt_acc: " << jnt_acc_des.transpose() << "\n"; */
  } else {
    matrix_t Qw =
        matrix_t::Zero(3 * (nc - numContacts_), 3 * (nc - numContacts_));
    swing_task.A.setZero(3 * (nc - numContacts_), numDecisionVars_);
    swing_task.b.setZero(swing_task.A.rows());
    auto base_pos_traj = refTrajBuffer_.get()->get_base_pos_ref_traj();

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
    swing_task.A = Qw * swing_task.A;
    swing_task.b = Qw * swing_task.b;
  }
  return swing_task;
}

void WholeBodyController::differential_inv_kin() {
  auto foot_traj_array = refTrajBuffer_.get()->get_foot_pos_traj();
  auto pos_traj = refTrajBuffer_.get()->get_base_pos_traj();

  if (foot_traj_array.empty()) {
    return;
  }

  int nj = static_cast<int>(actuated_joints_name.size());
  size_t nf = foot_names.size();
  auto contact_flag = quadruped::modeNumber2StanceLeg(mode_.get());
  scalar_t time_c = nodeHandle_->now().seconds() + 0.002;
  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  for (size_t k = 0; k < nf; k++) {
    const auto &foot_name = foot_names[k];
    matrix6x_t Jac_k;
    pinocchioInterface_ptr_->getJacobia_localWorldAligned(foot_name, Jac_k);
    vector<int> idx;
    for (int i = 0; i < nj; i++) {
      if (Jac_k.col(i + 6).head(3).norm() > 0.01) {
        idx.emplace_back(i);
      }
    }
    rcpputils::assert_true(idx.size() == 3);

    if (!contact_flag[k]) {
      const auto foot_traj = foot_traj_array[foot_name];
      matrix3_t Js_;
      vector3_t qpos_s;

      for (size_t i = 0; i < 3; i++) {
        Js_.col(i) = Jac_k.col(idx[i] + 6).head(3);
        qpos_s(i) = pinocchioInterface_ptr_->qpos()(7 + idx[i]);
      }
      matrix3_t J_inv = Js_.inverse();

      vector3_t pos_des, vel_des;
      vector3_t pos_m, vel_m;
      pos_m = (pinocchioInterface_ptr_->getFramePose(foot_name).translation() -
               base_pose.translation());
      vel_m =
          (pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(foot_name)
               .linear() -
           base_twist.linear());

      if (pos_traj.get() == nullptr) {
        pos_des = (foot_traj->evaluate(time_c) - base_pose.translation());
        vel_des = (foot_traj->derivative(time_c, 1) - base_twist.linear());
      } else {
        pos_des = (foot_traj->evaluate(time_c) - pos_traj->evaluate(time_c));
        vel_des = (foot_traj->derivative(time_c, 1) -
                   pos_traj->derivative(time_c, 1));
      }

      vector3_t pos_err = (pos_des - pos_m);
      if (pos_err.norm() > 0.1) {
        pos_err = 0.1 * pos_err.normalized();
      }
      vector3_t vel_err = (vel_des - vel_m);
      if (vel_err.norm() > 0.5) {
        vel_des = 0.5 * vel_err.normalized() + vel_m;
      }

      scalar_t kp_val, kd_val;
      if (pos_err.norm() < 3e-2) {
        kp_val = 60;
      } else {
        kp_val = 30;
      }
      if (vel_err.norm() < 0.1) {
        kd_val = 2;
      } else {
        kp_val = 1.0;
      }
      vector3_t q_des = J_inv * pos_err + qpos_s;
      vector3_t qd_des = J_inv * vel_des;
      for (size_t i = 0; i < 3; i++) {
        actuator_commands_->Kp(idx[i]) = kp_val;
        actuator_commands_->Kd(idx[i]) = kd_val;
        actuator_commands_->pos(idx[i]) = q_des(i);
        actuator_commands_->vel(idx[i]) = qd_des(i);
      }
    } else {
      if (joint_acc_.size() == nj) {
        vector_t jnt_pos = pinocchioInterface_ptr_->qpos().tail(nj);
        vector_t jnt_vel = pinocchioInterface_ptr_->qvel().tail(nj);
        for (size_t i = 0; i < 3; i++) {
          actuator_commands_->Kp(idx[i]) = 10.0;
          actuator_commands_->Kd(idx[i]) = 0.1;
          actuator_commands_->pos(idx[i]) =
              jnt_pos(idx[i]) + jnt_vel(idx[i]) * dt_ +
              0.5 * pow(dt_, 2) * joint_acc_(idx[i]);
          actuator_commands_->vel(idx[i]) =
              jnt_vel(idx[i]) + dt_ * joint_acc_(idx[i]);
        }
      }
    }
  }
  /* std::cout << "#####################################################\n";
  std::cout << "jnt kp: " <<  actuator_commands_->Kp.transpose() << "\n";
  std::cout << "jnt kd: " <<  actuator_commands_->Kd.transpose() << "\n"; */
}

MatrixDB WholeBodyController::formulateContactForceTask() {
  size_t nc = foot_names.size();
  size_t nv = pinocchioInterface_ptr_->nv();

  MatrixDB contact_force("contact_force");
  contact_force.A.setZero(3 * nc, numDecisionVars_);
  contact_force.b.setZero(contact_force.A.rows());
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
  weightBase_.setZero(6, 6);
  weightBase_.diagonal().fill(1000);

  weightSwingLeg_.setZero(3, 3);
  weightSwingLeg_.diagonal().fill(100);

  weightContactForce_ = 1e-9;

  frictionCoeff_ = 0.5;

  swingKp_.setZero(3, 3);
  swingKp_.diagonal().fill(500);

  swingKd_.setZero(3, 3);
  swingKd_.diagonal().fill(60);

  baseKp_.setZero(6, 6);
  baseKp_.diagonal().fill(200);

  baseKd_.setZero(6, 6);
  baseKd_.diagonal().fill(30);

  momentumKp_.setZero(6, 6);
  momentumKp_.diagonal().fill(0);

  momentumKd_.setZero(6, 6);
  momentumKd_.diagonal().fill(0);

  if (verbose) {
    std::cerr << "\n ########### weights.momentum ########### \n";
    std::cerr << weightMomentum_ << "\n";
    std::cerr << "\n ########### weights.floatingbase ########### \n";
    std::cerr << weightBase_ << "\n";
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
    std::cerr << baseKp_ << "\n";
    std::cerr << "\n ########### feedback_gain.floatingbase.kd ########### \n";
    std::cerr << baseKd_ << "\n";
    std::cerr << "\n ########### feedback_gain.momentum.kp ########### \n";
    std::cerr << momentumKp_ << "\n";
    std::cerr << "\n ########### feedback_gain.momentum.kd ########### \n";
    std::cerr << momentumKd_ << "\n";
  }
}

vector3_t WholeBodyController::compute_euler_angle_err(const vector3_t &rpy_m,
                                                       const vector3_t &rpy_d) {
  vector3_t rpy_err = rpy_m - rpy_d;
  if (rpy_err.norm() > 1.5 * M_PI) {
    if (abs(rpy_err(0)) > M_PI) {
      rpy_err(0) += (rpy_err(0) > 0 ? -2.0 : 2.0) * M_PI;
    }
    if (abs(rpy_err(1)) > M_PI) {
      rpy_err(1) += (rpy_err(1) > 0 ? -2.0 : 2.0) * M_PI;
    }
    if (abs(rpy_err(2)) > M_PI) {
      rpy_err(2) += (rpy_err(2) > 0 ? -2.0 : 2.0) * M_PI;
    }
  }
  return rpy_err;
}

} // namespace clear