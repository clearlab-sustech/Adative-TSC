#include "trajectory/TrajectoriesArray.h"

namespace clear {

TrajectoriesArray::TrajectoriesArray() {}

TrajectoriesArray::~TrajectoriesArray() {}

void TrajectoriesArray::clear_all() {
  base_rpy_buffer_.clear();
  base_pos_buffer_.clear();
  foot_rpy_buffer_.clear();
  foot_pos_buffer_.clear();
  gripper_rpy_buffer_.clear();
  gripper_pos_buffer_.clear();
  com_pos_buffer_.clear();
  angular_momentum_buffer_.clear();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_base_rpy_traj() const {
  return base_rpy_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_base_pos_traj() const {
  if (base_pos_buffer_.get().get() != nullptr) {
    return base_pos_buffer_.get();
  } else {
    return base_pos_ref_buffer_.get();
  }
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_base_pos_ref_traj() const {
  return base_pos_ref_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
TrajectoriesArray::get_foot_rpy_traj() const {
  return foot_rpy_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
TrajectoriesArray::get_foot_pos_traj() const {
  return foot_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_gripper_rpy_traj() const {
  return gripper_rpy_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_gripper_pos_traj() const {
  return gripper_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_com_pos_traj() const {
  return com_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_angular_momentum_traj() const {
  return angular_momentum_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_joint_pos_traj() const {
  return joint_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_optimized_base_pos_traj() const {
  return optimized_base_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
TrajectoriesArray::get_optimized_base_rpy_traj() const {
  return optimized_base_rpy_buffer_.get();
}

void TrajectoriesArray::set_base_rpy_traj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  base_rpy_buffer_.push(base_rpy_traj);
}

void TrajectoriesArray::set_base_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  base_pos_buffer_.push(base_pos_traj);
}

void TrajectoriesArray::set_optimized_base_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  optimized_base_pos_buffer_.push(base_pos_traj);
}

void TrajectoriesArray::set_optimized_base_rpy_traj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  optimized_base_rpy_buffer_.push(base_rpy_traj);
}

void TrajectoriesArray::set_base_pos_ref_traj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_ref_traj) {
  base_pos_ref_buffer_.push(base_pos_ref_traj);
}

void TrajectoriesArray::set_foot_rpy_traj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_rpy_traj) {
  foot_rpy_buffer_.push(foot_rpy_traj);
}

void TrajectoriesArray::set_foot_pos_traj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_pos_traj) {
  foot_pos_buffer_.push(foot_pos_traj);
}

void TrajectoriesArray::set_gripper_rpy_traj(
    std::shared_ptr<CubicSplineTrajectory> gripper_rpy_traj) {
  gripper_rpy_buffer_.push(gripper_rpy_traj);
}

void TrajectoriesArray::set_gripper_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> gripper_pos_traj) {
  gripper_pos_buffer_.push(gripper_pos_traj);
}

void TrajectoriesArray::set_com_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> com_pos_traj) {
  com_pos_buffer_.push(com_pos_traj);
}

void TrajectoriesArray::set_angular_momentum_traj(
    std::shared_ptr<CubicSplineTrajectory> angular_momentum_traj) {
  angular_momentum_buffer_.push(angular_momentum_traj);
}

void TrajectoriesArray::set_joint_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> joint_pos_traj) {
  joint_pos_buffer_.push(joint_pos_traj);
}

} // namespace clear