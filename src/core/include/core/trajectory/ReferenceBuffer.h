#pragma once
#include "core/gait/ModeSchedule.h"
#include "core/misc/Buffer.h"
#include "core/trajectory/CubicSplineTrajectory.h"

#include <map>
#include <memory>

namespace clear {
class ReferenceBuffer {
public:
  ReferenceBuffer();

  ~ReferenceBuffer();

  void clearAll();

  std::shared_ptr<CubicSplineTrajectory> getIntegratedBaseRpyTraj() const;

  std::shared_ptr<CubicSplineTrajectory> getIntegratedBasePosTraj() const;

  std::shared_ptr<CubicSplineTrajectory> getOptimizedBasePosTraj() const;

  std::shared_ptr<CubicSplineTrajectory> getOptimizedBaseRpyTraj() const;

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
  getFootRpyTraj() const;

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
  getFootPosTraj() const;

  std::shared_ptr<CubicSplineTrajectory> getJointsPosTraj() const;

  std::map<std::string, std::pair<scalar_t, vector3_t>> getFootholds() const;

  std::shared_ptr<ModeSchedule> getModeSchedule() const;

  void setIntegratedBaseRpyTraj(
      std::shared_ptr<CubicSplineTrajectory> base_rpy_traj);

  void setIntegratedBasePosTraj(
      std::shared_ptr<CubicSplineTrajectory> base_pos_traj);

  void
  setOptimizedBasePosTraj(std::shared_ptr<CubicSplineTrajectory> base_pos_traj);

  void
  setOptimizedBaseRpyTraj(std::shared_ptr<CubicSplineTrajectory> base_rpy_traj);

  void
  setFootRpyTraj(std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
                     foot_rpy_traj);

  void
  setFootPosTraj(std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
                     foot_pos_traj);

  void setJointsPosTraj(std::shared_ptr<CubicSplineTrajectory> joints_pos_traj);

  void
  setFootholds(std::map<std::string, std::pair<scalar_t, vector3_t>> footholds);

  void setModeSchedule(std::shared_ptr<ModeSchedule> mode_schedule);

private:
  Buffer<std::shared_ptr<CubicSplineTrajectory>> integ_base_rpy_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> integ_base_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> optimized_base_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> optimized_base_rpy_buffer_;
  Buffer<std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>>
      foot_rpy_buffer_;
  Buffer<std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>>
      foot_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> joints_pos_buffer_;
  Buffer<std::map<std::string, std::pair<scalar_t, vector3_t>>>
      footholds_buffer_;
  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer_;
};

} // namespace clear