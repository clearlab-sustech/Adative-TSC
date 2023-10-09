#pragma once
#include <core/misc/Buffer.h>
#include <core/trajectory/CubicSplineTrajectory.h>

#include <map>
#include <memory>

namespace clear {
class TrajectoriesArray {
public:
  TrajectoriesArray();

  ~TrajectoriesArray();

  void clear_all();

  std::shared_ptr<CubicSplineTrajectory> get_base_rpy_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_base_pos_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_base_pos_ref_traj() const;

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
  get_foot_rpy_traj() const;

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
  get_foot_pos_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_gripper_rpy_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_gripper_pos_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_com_pos_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_angular_momentum_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_joint_pos_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_optimized_base_pos_traj() const;

  std::shared_ptr<CubicSplineTrajectory> get_optimized_base_rpy_traj() const;

  void set_base_rpy_traj(std::shared_ptr<CubicSplineTrajectory> base_rpy_traj);

  void set_base_pos_traj(std::shared_ptr<CubicSplineTrajectory> base_pos_traj);

  void set_optimized_base_pos_traj(
      std::shared_ptr<CubicSplineTrajectory> base_pos_traj);

  void set_optimized_base_rpy_traj(
      std::shared_ptr<CubicSplineTrajectory> base_rpy_traj);

  void set_base_pos_ref_traj(
      std::shared_ptr<CubicSplineTrajectory> base_pos_ref_traj);

  void set_foot_rpy_traj(
      std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
          foot_rpy_traj);

  void set_foot_pos_traj(
      std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
          foot_pos_traj);

  void
  set_gripper_rpy_traj(std::shared_ptr<CubicSplineTrajectory> gripper_rpy_traj);

  void
  set_gripper_pos_traj(std::shared_ptr<CubicSplineTrajectory> gripper_pos_traj);

  void set_com_pos_traj(std::shared_ptr<CubicSplineTrajectory> com_pos_traj);

  void set_angular_momentum_traj(
      std::shared_ptr<CubicSplineTrajectory> angular_momentum_traj);

  void
  set_joint_pos_traj(std::shared_ptr<CubicSplineTrajectory> joint_pos_traj);

private:
  Buffer<std::shared_ptr<CubicSplineTrajectory>> base_rpy_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> base_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> optimized_base_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> optimized_base_rpy_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> base_pos_ref_buffer_;
  Buffer<std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>>
      foot_rpy_buffer_;
  Buffer<std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>>
      foot_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> gripper_rpy_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> gripper_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> com_pos_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> angular_momentum_buffer_;
  Buffer<std::shared_ptr<CubicSplineTrajectory>> joint_pos_buffer_;
};

} // namespace clear
