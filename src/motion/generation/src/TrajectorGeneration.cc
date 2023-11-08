#include "generation/TrajectorGeneration.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear
{

  TrajectorGeneration::TrajectorGeneration(Node::SharedPtr nodeHandle,
                                           string config_yaml)
      : nodeHandle_(nodeHandle)
  {
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

    std::string ocs2_leg_robot_package = config_["ocs2"]["package"].as<std::string>();
    std::string reference_file =
        ament_index_cpp::get_package_share_directory(ocs2_leg_robot_package) +
        config_["ocs2"]["reference_file"].as<std::string>();
    RCLCPP_INFO(nodeHandle_->get_logger(), "reference file: %s", reference_file.c_str());

    std::string task_file =
        ament_index_cpp::get_package_share_directory(ocs2_leg_robot_package) +
        config_["ocs2"]["task_file"].as<std::string>();
    RCLCPP_INFO(nodeHandle_->get_logger(), "task file: %s", task_file.c_str());

    robot_interface_ptr_ = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(task_file, urdf, reference_file);
    gait_receiver_ptr_ = std::make_shared<ocs2::legged_robot::GaitReceiver>(
        nodeHandle_,
        robot_interface_ptr_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
        robot_name);
    reference_manager_ptr_ = std::make_shared<ocs2::RosReferenceManager>(
        robot_name, robot_interface_ptr_->getReferenceManagerPtr());
    reference_manager_ptr_->subscribe(nodeHandle_);

    mpc_ptr_ = std::make_shared<ocs2::SqpMpc>(robot_interface_ptr_->mpcSettings(), robot_interface_ptr_->sqpSettings(),
                                              robot_interface_ptr_->getOptimalControlProblem(), robot_interface_ptr_->getInitializer());
    mpc_ptr_->getSolverPtr()->setReferenceManager(reference_manager_ptr_);
    mpc_ptr_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr_);

    refTrajBuffer_ = std::make_shared<TrajectoriesArray>();

    vel_cmd.setZero();
    yawd_ = 0.0;

    run_.push(true);
    inner_loop_thread_ = std::thread(&TrajectorGeneration::inner_loop, this);
  }

  TrajectorGeneration::~TrajectorGeneration()
  {
    run_.push(false);
    inner_loop_thread_.join();
  }

  void TrajectorGeneration::update_current_state(
      std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr)
  {
    qpos_ptr_buffer.push(qpos_ptr);
    qvel_ptr_buffer.push(qvel_ptr);
  }

  std::shared_ptr<TrajectoriesArray>
  TrajectorGeneration::get_trajectory_reference()
  {
    return refTrajBuffer_;
  }

  void TrajectorGeneration::inner_loop()
  {
    benchmark::RepeatedTimer timer_;
    rclcpp::Rate loop_rate(freq_);
    while (rclcpp::ok() && run_.get())
    {
      timer_.startTimer();
      if (qpos_ptr_buffer.get().get() == nullptr ||
          qvel_ptr_buffer.get().get() == nullptr)
      {
        continue;
      }
      else
      {
        std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
        std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      }
      timer_.endTimer();
      loop_rate.sleep();
    }
    RCLCPP_INFO(nodeHandle_->get_logger(),
                "[TrajectorGeneration] max time %f ms,  average time %f ms",
                timer_.getMaxIntervalInMilliseconds(),
                timer_.getAverageInMilliseconds());
  }

  void TrajectorGeneration::setVelCmd(vector3_t vd, scalar_t yawd)
  {
    vel_cmd = vd;
    yawd_ = yawd;
  }

} // namespace clear
