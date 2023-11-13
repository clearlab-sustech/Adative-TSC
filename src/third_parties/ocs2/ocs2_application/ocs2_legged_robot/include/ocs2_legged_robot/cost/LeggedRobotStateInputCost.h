/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot/common/utils.h"
#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/cost/StateInputCostCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {
namespace legged_robot {

/**
 * State-input tracking cost used for intermediate times
 */
class LeggedRobotStateInputCost : public StateInputCostCppAd {
public:
  LeggedRobotStateInputCost(
      matrix_t Q, matrix_t R, CentroidalModelInfo info,
      const SwitchedModelReferenceManager &referenceManager,
      const PinocchioInterface &pinocchioInterface, const std::string &name,
      const std::string &cppAdModelFolder = "/tmp/ocs2",
      bool recompileLibraries = true, bool verbose = false);

  ~LeggedRobotStateInputCost() override = default;

  LeggedRobotStateInputCost *clone() const override;

  virtual vector_t
  getParameters(scalar_t time, const TargetTrajectories &targetTrajectories,
                const PreComputation & /* preComputation */) const;

protected:
  LeggedRobotStateInputCost(const LeggedRobotStateInputCost &rhs) = default;

  /** The CppAD cost function */
  virtual ad_scalar_t costFunction(ad_scalar_t time, const ad_vector_t &state,
                                   const ad_vector_t &input,
                                   const ad_vector_t &parameters) const;

private:
  matrix_t Q_;
  matrix_t R_;
  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager *referenceManagerPtr_;
  mutable PinocchioInterfaceCppAd pinocchioInterfaceCppAd;
};

} // namespace legged_robot
} // namespace ocs2
