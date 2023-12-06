#include "asserts/trajectory/ReferenceBuffer.h"

namespace clear {

ReferenceBuffer::ReferenceBuffer() {}

ReferenceBuffer::~ReferenceBuffer() {}

void ReferenceBuffer::clear_all() {
  base_rpy_buffer_.clear();
  base_pos_buffer_.clear();
  foot_rpy_buffer_.clear();
  foot_pos_buffer_.clear();
  com_pos_buffer_.clear();
  angular_momentum_buffer_.clear();
  footholds_buffer_.clear();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::get_base_rpy_traj() const {
  return base_rpy_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::get_base_pos_traj() const {
  return base_pos_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::get_foot_rpy_traj() const {
  return foot_rpy_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::get_foot_pos_traj() const {
  return foot_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::get_com_pos_traj() const {
  return com_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::get_angular_momentum_traj() const {
  return angular_momentum_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::get_joint_pos_traj() const {
  return joint_pos_buffer_.get();
}

std::map<std::string, std::pair<scalar_t, vector3_t>>
ReferenceBuffer::get_footholds() const {
  return footholds_buffer_.get();
}

std::shared_ptr<ModeSchedule> ReferenceBuffer::get_mode_schedule() const {
  return mode_schedule_buffer_.get();
}

void ReferenceBuffer::set_base_rpy_traj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  base_rpy_buffer_.push(base_rpy_traj);
}

void ReferenceBuffer::set_base_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  base_pos_buffer_.push(base_pos_traj);
}

void ReferenceBuffer::set_foot_rpy_traj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_rpy_traj) {
  foot_rpy_buffer_.push(foot_rpy_traj);
}

void ReferenceBuffer::set_foot_pos_traj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_pos_traj) {
  foot_pos_buffer_.push(foot_pos_traj);
}

void ReferenceBuffer::set_com_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> com_pos_traj) {
  com_pos_buffer_.push(com_pos_traj);
}

void ReferenceBuffer::set_angular_momentum_traj(
    std::shared_ptr<CubicSplineTrajectory> angular_momentum_traj) {
  angular_momentum_buffer_.push(angular_momentum_traj);
}

void ReferenceBuffer::set_joint_pos_traj(
    std::shared_ptr<CubicSplineTrajectory> joint_pos_traj) {
  joint_pos_buffer_.push(joint_pos_traj);
}

void ReferenceBuffer::set_footholds(
    std::map<std::string, std::pair<scalar_t, vector3_t>> footholds) {
  footholds_buffer_.push(footholds);
}

void ReferenceBuffer::set_mode_schedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer_.push(mode_schedule);
}

} // namespace clear