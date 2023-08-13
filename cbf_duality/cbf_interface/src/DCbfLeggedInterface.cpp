//
// Created by qiayuan on 2022/7/16.
//

#include <memory>

#include "cbf_interface/DCbfLeggedInterface.h"
#include "cbf_interface/Duality.h"
#include "cbf_interface/DualityInitializer.h"
#include "cbf_interface/FactoryAccessHelper.h"

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

namespace cbf {

void DualityLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                        const std::string& referenceFile, bool verbose) {
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  setupObstacles();
  //  std::unique_ptr<PenaltyBase> penalty_lag(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(5.0, 1e-2)));
  std::unique_ptr<PenaltyBase> penalty_lag(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.01, 1e-3)));
  std::unique_ptr<PenaltyBase> penalty_positive(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));

  std::unique_ptr<DualityLagrangianAd> lagrangian(new DualityLagrangianAd(*obstacles_, dualityInfo_));
  getOptimalControlProblem().softConstraintPtr->add(
      "duality_lagrangian", std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(lagrangian), std::move(penalty_lag))));

  std::unique_ptr<DualityPositiveAd> positive(new DualityPositiveAd(*obstacles_, dualityInfo_));
  getOptimalControlProblem().softConstraintPtr->add(
      "duality_positive", std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(positive), std::move(penalty_positive))));

  std::unique_ptr<DualityVectorAd> equality(new DualityVectorAd(*obstacles_, dualityInfo_));
  getOptimalControlProblem().equalityConstraintPtr->add("duality_vector", std::move(equality));

  std::unique_ptr<DualityNormAd> norm(new DualityNormAd(*obstacles_, dualityInfo_));
  getOptimalControlProblem().equalityConstraintPtr->add("duality_norm", std::move(norm));

  getInitializerPtr() = std::make_unique<DualityInitializer>(dualityInfo_, *getSwitchedModelReferenceManagerPtr(), *obstacles_);
}

void DualityLeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                        bool verbose) {
  LeggedInterface::setupModel(taskFile, urdfFile, referenceFile, verbose);

  dualityInfo_ = createDualityInfo(getCentroidalModelInfo(), 1, 6, 9);// TODO
  getCentroidalModelInfo() = dualityInfo_.centroidalInfo;
}

void DCbfDualityLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                            const std::string& referenceFile, bool verbose) {
  DualityLeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  getOptimalControlProblem().softConstraintPtr->erase("duality_lagrangian");

  std::unique_ptr<PenaltyBase> penalty_lag(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.01, 1e-3)));
  std::unique_ptr<DCbfLagrangianAd> lagrangian(new DCbfLagrangianAd(*obstacles_, dualityInfo_));
  getOptimalControlProblem().softConstraintPtr->add(
      "dcbf_lagrangian", std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(lagrangian), std::move(penalty_lag))));
}

}  // namespace cbf
