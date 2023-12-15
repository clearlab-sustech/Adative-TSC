#include "core/trajectory/ReferenceBuffer.h"

namespace clear {

ReferenceBuffer::ReferenceBuffer() {}

ReferenceBuffer::~ReferenceBuffer() {}

void ReferenceBuffer::clearAll() {
  integ_base_rpy_buffer_.clear();
  integ_base_pos_buffer_.clear();
  optimized_base_pos_buffer_.clear();
  optimized_base_rpy_buffer_.clear();
  foot_rpy_buffer_.clear();
  foot_pos_buffer_.clear();
  joints_pos_buffer_.clear();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getIntegratedBaseRpyTraj() const {
  return integ_base_rpy_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getIntegratedBasePosTraj() const {
  return integ_base_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getOptimizedBaseRpyTraj() const {
  return optimized_base_rpy_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getOptimizedBasePosTraj() const {
  return optimized_base_pos_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::getFootRpyTraj() const {
  return foot_rpy_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::getFootPosTraj() const {
  return foot_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getJointsPosTraj() const {
  return joints_pos_buffer_.get();
}

std::map<std::string, std::pair<scalar_t, vector3_t>>
ReferenceBuffer::getFootholds() const {
  return footholds_buffer_.get();
}

std::shared_ptr<ModeSchedule> ReferenceBuffer::getModeSchedule() const {
  return mode_schedule_buffer_.get();
}

void ReferenceBuffer::setIntegratedBaseRpyTraj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  integ_base_rpy_buffer_.push(base_rpy_traj);
}

void ReferenceBuffer::setIntegratedBasePosTraj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  integ_base_pos_buffer_.push(base_pos_traj);
}

void ReferenceBuffer::setOptimizedBasePosTraj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  optimized_base_pos_buffer_.push(base_pos_traj);
}

void ReferenceBuffer::setOptimizedBaseRpyTraj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  optimized_base_rpy_buffer_.push(base_rpy_traj);
}

void ReferenceBuffer::setFootRpyTraj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_rpy_traj) {
  foot_rpy_buffer_.push(foot_rpy_traj);
}

void ReferenceBuffer::setFootPosTraj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_pos_traj) {
  foot_pos_buffer_.push(foot_pos_traj);
}

void ReferenceBuffer::setJointsPosTraj(
    std::shared_ptr<CubicSplineTrajectory> joints_pos_traj) {
  joints_pos_buffer_.push(joints_pos_traj);
}

void ReferenceBuffer::setFootholds(
    std::map<std::string, std::pair<scalar_t, vector3_t>> footholds) {
  footholds_buffer_.push(footholds);
}

void ReferenceBuffer::setModeSchedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer_.push(mode_schedule);
}

} // namespace clear