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

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Dummy_Loop::MRT_ROS_Dummy_Loop(Node::SharedPtr nodeHandle,
                                       MRT_ROS_Interface &mrt,
                                       scalar_t mrtDesiredFrequency,
                                       scalar_t mpcDesiredFrequency)
    : nodeHandle_(nodeHandle), mrt_(mrt),
      mrtDesiredFrequency_(mrtDesiredFrequency),
      mpcDesiredFrequency_(mpcDesiredFrequency) {
  if (mrtDesiredFrequency_ < 0) {
    throw std::runtime_error("MRT loop frequency should be a positive number.");
  }

  if (mpcDesiredFrequency_ > 0) {
    RCLCPP_WARN_STREAM(nodeHandle_->get_logger(),
                       "MPC loop is not realtime! For realtime setting, set "
                       "mpcDesiredFrequency to any negative number.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::run(
    const SystemObservation &initObservation_ref,
    const TargetTrajectories &initTargetTrajectories_ref) {
  initObservation = initObservation_ref;
  initTargetTrajectories = initTargetTrajectories_ref;

  RCLCPP_WARN_STREAM(nodeHandle_->get_logger(),
                     "Waiting for the initial policy ...");

  // Reset MPC node
  mrt_.resetMpc(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt_.initialPolicyReceived() && rclcpp::ok()) {
    rclcpp::spin_some(nodeHandle_);
    mrt_.setCurrentObservation(initObservation);
    rclcpp::Rate(mrtDesiredFrequency_).sleep();
  }
  RCLCPP_INFO_STREAM(nodeHandle_->get_logger(),
                     "Initial policy has been received.");

  currentObservation = initObservation;

  // Pick simulation loop mode
  if (mpcDesiredFrequency_ > 0.0) {
    synchronizedDummyLoop();
  } else {
    realtimeDummyLoop();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::synchronizedDummyLoop() {
  rclcpp::Rate simRate(mrtDesiredFrequency_);
  while (rclcpp::ok()) {
    // Determine the ratio between MPC updates and simulation steps.
    const auto mpcUpdateRatio = std::max(
        static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_),
        size_t(1));

    // Helper function to check if policy is updated and starts at the given
    // time. Due to ROS message conversion delay and very fast MPC loop, we
    // might get an old policy instead of the latest one.
    const auto policyUpdatedForTime = [this](scalar_t time) {
      constexpr scalar_t tol =
          0.1; // policy must start within this fraction of dt
      return mrt_.updatePolicy() &&
             std::abs(mrt_.getPolicy().timeTrajectory_.front() - time) <
                 (tol / mpcDesiredFrequency_);
    };
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0) {
      // Wait for the policy to be updated
      while (!policyUpdatedForTime(currentObservation.time) && rclcpp::ok()) {
        rclcpp::spin_some(nodeHandle_);
      }
      std::cout << "<<< New MPC policy starting at "
                << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // Forward simulation
    currentObservation = forwardSimulation();

    // User-defined modifications before publishing
    modifyObservation();

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0) {
      mrt_.setCurrentObservation(currentObservation);
      std::cout << ">>> Observation is published at " << currentObservation.time
                << "\n";
    }

    // Update observers
    for (auto &observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ++loopCounter;
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::modifyObservation() {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::realtimeDummyLoop() {
  while (rclcpp::ok()) {
    std::cout << "### Current time " << currentObservation.time << "\n";
    // Trigger MRT callbacks
    rclcpp::spin_some(nodeHandle_);

    // Update the policy if a new on was received
    if (mrt_.updatePolicy()) {
      std::cout << "<<< New MPC policy starting at "
                << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // Forward simulation
    currentObservation = forwardSimulation();

    // User-defined modifications before publishing
    modifyObservation();

    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    // Update observers
    for (auto &observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Dummy_Loop::forwardSimulation() {
  const scalar_t dt = 1.0 / mrtDesiredFrequency_;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt;
  if (mrt_.isRolloutSet()) { // If available, use the provided rollout as to
                             // integrate the dynamics.
    mrt_.rolloutPolicy(currentObservation.time, currentObservation.state, dt,
                       nextObservation.state, nextObservation.input,
                       nextObservation.mode);
  } else { // Otherwise, we fake integration by interpolating the current MPC
           // policy at t+dt
    mrt_.evaluatePolicy(currentObservation.time + dt, currentObservation.state,
                        nextObservation.state, nextObservation.input,
                        nextObservation.mode);
  }

  return nextObservation;
}

} // namespace ocs2
