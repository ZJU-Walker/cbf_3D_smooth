//
// Created by Ke Wang on 2023/7/20.
//
#pragma once
#include <utility>

#include "cbf_interface/Duality.h"

namespace cbf {
using namespace ocs2;

class DCbfLagrangianAd : public DualityBaseAd {
 public:
  DCbfLagrangianAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DCbfLagrangian", "/tmp/ocs2",
               true, false);
  }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { 
    return info_.numRobots * info_.numObstacles; 
  }

  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    scalar_t decay_rate = 1.0;

    auto robot = createRobotRegion3d(state, info_.centroidalInfo);

    ad_vector_t constraint(info_.numRobots * info_.numObstacles);

    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (size_t r = 0; r < info_.numRobots; ++r) {
        constraint(o * info_.numRobots + r) = (-getRobotLambda3d(input, info_, o, r).transpose() * robot[r].getB() -
                       getObstacleLambda3d(input, info_, o, r).transpose() * getObstacleB(parameters, o))(0) -
                      (0.05 + exp(-decay_rate * (time - getObstacleTime(parameters))) * getObstacleDist(parameters, o, r));
      }
    }

    return constraint;
  }
  
  DCbfLagrangianAd* clone() const override { return new DCbfLagrangianAd(*this); }

 protected:
  ad_scalar_t getObstacleTime(const ad_vector_t& parameters) const { return parameters(4 * info_.numObstacles * 6); }

  ad_scalar_t getObstacleDist(const ad_vector_t& parameters, size_t o, size_t r) const {
    return parameters(4 * info_.numObstacles * 6 + 1 + o * info_.numRobots + r);
  }

 private:
  DCbfLagrangianAd(const DualityObstacles& obstacles, DCbfLagrangianAd rhs) : DCbfLagrangianAd(std::move(rhs)) {}
};

}  // namespace cbf
