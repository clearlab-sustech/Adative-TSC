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
#include "ocs2_legged_robot/cost/LeggedRobotStateInputCost.h"
#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2 {

namespace legged_robot {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotStateInputCost::LeggedRobotStateInputCost(
    matrix_t Q, matrix_t R, CentroidalModelInfo info,
    const SwitchedModelReferenceManager &referenceManager,
    const PinocchioInterface &pinocchioInterface, const std::string &name,
    const std::string &cppAdModelFolder, bool recompileLibraries, bool verbose)
    : StateInputCostCppAd(), Q_(std::move(Q)), R_(std::move(R)), info_(info),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd(pinocchioInterface.toCppAd()) {

  StateInputCostCppAd::initialize(
      info_.stateDim, info_.inputDim, info_.stateDim + info_.inputDim, name,
      cppAdModelFolder, recompileLibraries, verbose);
}

LeggedRobotStateInputCost *LeggedRobotStateInputCost::clone() const {
  return new LeggedRobotStateInputCost(*this);
}

vector_t LeggedRobotStateInputCost::getParameters(
    scalar_t time, const TargetTrajectories &targetTrajectories,
    const PreComputation & /* preComputation */) const {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);

  vector_t nominal_traj(info_.stateDim + info_.inputDim);
  nominal_traj.head(info_.stateDim) = targetTrajectories.getDesiredState(time);
  nominal_traj.tail(info_.inputDim) =
      weightCompensatingInput(info_, contactFlags);

  return nominal_traj;
}

ad_scalar_t
LeggedRobotStateInputCost::costFunction(ad_scalar_t, const ad_vector_t &state,
                                        const ad_vector_t &input,
                                        const ad_vector_t &parameters) const {

  const auto infoCppAd_ = info_.toCppAd();
  const ad_vector_t q =
      centroidal_model::getGeneralizedCoordinates(state, infoCppAd_);

  updateCentroidalDynamics(pinocchioInterfaceCppAd, infoCppAd_, q);

  auto pinocchioMappingCppAd =
      std::make_shared<CentroidalModelPinocchioMappingCppAd>(info_.toCppAd());

  pinocchioMappingCppAd->setPinocchioInterface(pinocchioInterfaceCppAd);

  ad_vector_t state_ = state;

  state_.head(3) =
      pinocchioMappingCppAd->getPinocchioJointVelocity(state, input).head(3);

  const ad_vector_t stateDeviation =
      state_ - parameters.head(infoCppAd_.stateDim);
  const ad_vector_t inputDeviation =
      input - parameters.tail(infoCppAd_.inputDim);

  ad_matrix_t Q_ad = Q_.cast<ad_scalar_t>();
  ad_matrix_t R_ad = R_.cast<ad_scalar_t>();

  return (0.5 * stateDeviation.dot(Q_ad * stateDeviation) +
          0.5 * inputDeviation.dot(R_ad * inputDeviation));
}

} // namespace legged_robot

} // namespace ocs2
