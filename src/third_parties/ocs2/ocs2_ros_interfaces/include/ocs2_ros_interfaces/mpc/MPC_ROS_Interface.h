/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_msgs/srv/reset.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include <rclcpp/rclcpp.hpp>
using namespace rclcpp;

#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MPC communication interface using ROS.
 */
class MPC_ROS_Interface {
public:
  typedef ocs2_msgs::msg::MpcObservation::ConstSharedPtr MpcObservationPtr;
  typedef std::function<void(const MpcObservationPtr)> MpcObservationCallback;

  /**
   * Constructor.
   *
   * @param [in] mpc: The underlying MPC class to be used.
   * @param [in] topicPrefix: The robot's name.
   */
  explicit MPC_ROS_Interface(Node::SharedPtr nodeHandle, MPC_BASE &mpc,
                             std::string topicPrefix = "anonymousRobot");

  /**
   * Destructor.
   */
  virtual ~MPC_ROS_Interface();

  /**
   * Resets the class to its instantiation state.
   *
   * @param [in] initTargetTrajectories: The initial desired cost trajectories.
   */
  void resetMpc(TargetTrajectories &&initTargetTrajectories);

  /**
   * Shutdowns
   */
  void shutdown();

protected:
  /**
   * Callback to reset MPC.
   *
   * @param req: Service request.
   * @param res: Service response.
   */
  bool
  resetMpcCallback(const std::shared_ptr<ocs2_msgs::srv::Reset::Request> req,
                   std::shared_ptr<ocs2_msgs::srv::Reset::Response> res);

  /**
   * Creates MPC Policy message.
   *
   * @param [in] primalSolution: The policy data of the MPC.
   * @param [in] commandData: The command data of the MPC.
   * @param [in] performanceIndices: The performance indices data of the solver.
   * @return MPC policy message.
   */
  static ocs2_msgs::msg::MpcFlattenedController
  createMpcPolicyMsg(const PrimalSolution &primalSolution,
                     const CommandData &commandData,
                     const PerformanceIndex &performanceIndices);

  /**
   * Handles ROS publishing thread.
   */
  void publisherWorker();

  /**
   * Updates the buffer variables from the MPC object. This method is
   * automatically called by advanceMpc()
   *
   * @param [in] mpcInitObservation: The observation used to run the MPC.
   */
  void copyToBuffer(const SystemObservation &mpcInitObservation) const;

private:
  /**
   * The callback method which receives the current observation, invokes the MPC
   * algorithm, and finally publishes the optimized policy.
   *
   * @param [in] msg: The observation message.
   */
  void mpcObservationCallback(
      const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg);

protected:
  /*
   * Variables
   */
  Node::SharedPtr nodeHandle_;

  MPC_BASE &mpc_;

  std::string topicPrefix_;

  // Publishers and subscribers
  Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      mpcObservationSubscriber_;
  Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
      mpcTargetTrajectoriesSubscriber_;
  Publisher<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr
      mpcPolicyPublisher_;
  Service<ocs2_msgs::srv::Reset>::SharedPtr mpcResetServiceServer_;

  std::unique_ptr<CommandData> bufferCommandPtr_;
  std::unique_ptr<CommandData> publisherCommandPtr_;
  std::unique_ptr<PrimalSolution> bufferPrimalSolutionPtr_;
  std::unique_ptr<PrimalSolution> publisherPrimalSolutionPtr_;
  std::unique_ptr<PerformanceIndex> bufferPerformanceIndicesPtr_;
  std::unique_ptr<PerformanceIndex> publisherPerformanceIndicesPtr_;

  mutable std::mutex bufferMutex_; // for policy variables with prefix (buffer*)

  // multi-threading for publishers
  mutable std::atomic_bool terminateThread_{false};
  mutable std::atomic_bool readyToPublish_{false};
  mutable std::thread publisherWorker_;
  mutable std::mutex publisherMutex_;
  mutable std::condition_variable msgReady_;

  mutable benchmark::RepeatedTimer mpcTimer_;

  // MPC reset
  mutable std::mutex resetMutex_;
  mutable std::atomic_bool resetRequestedEver_{false};
};

} // namespace ocs2
