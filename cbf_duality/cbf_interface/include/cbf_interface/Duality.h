//
// Created by Ke Wang on 2023/7/20.
//
#pragma once
#include "cbf_geometry/ConvexRegion3d.h"
#include "cbf_interface/FactoryAccessHelper.h"
#include "cbf_interface/Obstacle.h"

#include <cbf_geometry/ConvexRegion3d.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include <utility>

namespace cbf {
using namespace ocs2;
// Base, define function get obstacle A and B from parameters
class DualityBaseAd : public StateInputConstraintCppAd {
 public:
  DualityBaseAd(const DualityObstacles& obstacles, DualityInfo info)
      : StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), obstacles_(obstacles), info_(std::move(info)) {}

   // Get parameters (A and B)
  vector_t getParameters(scalar_t /*time*/, const PreComputation& /* preComputation */) const override {
    return obstacles_.getParameters();
  };

 protected:
  // Get obstacle matrix A (6 x 3)
  ad_matrix_t getObstacleA(const ad_vector_t& parameters, size_t o) const {
    ad_matrix_t ret(6, 3);
    for (int p = 0; p < 6; ++p) {
      ret.row(p) = parameters.segment<3>(o * 4 * 6 + p * 3);
    }
    return ret;
  }

   // Get obstacle vector B (6 x 1)
  ad_vector_t getObstacleB(const ad_vector_t& parameters, size_t o) const {
    return parameters.segment(o * 4 * 6 + 6 * 3, 6);
  }

   // Copy constructor
  DualityBaseAd(const DualityObstacles& obstacles, const DualityBaseAd& rhs)
      : StateInputConstraintCppAd(rhs), obstacles_(obstacles), info_(rhs.info_) {}

  const DualityObstacles& obstacles_;
  const DualityInfo info_;
};




// This class represents a Duality Lagrangian with additional constraints
// 10(a)
class DualityLagrangianAd : public DualityBaseAd {
 public:
  // Constructor that initializes the Duality Lagrangian with obstacles and information
  DualityLagrangianAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    // Initialize the base class with the centroidal information
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityLagrangian", "/tmp/ocs2",
               true, false);
  }

   // Get the number of constraints
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { 
    return info_.numRobots * info_.numObstacles; 
  }

  // Calculate the constraint function for a given time, state, input, and parameters
  ad_vector_t constraintFunction(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    // Create a robot region based on the state and centroidal information
    auto robot = createRobotRegion3d(state, info_.centroidalInfo);// info_.numRobots x 1 vector
    // Calculate the constraint for each obstacle
    ad_vector_t constraint(info_.numRobots * info_.numObstacles);//numObstacles x info_.numRobots

    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (size_t r = 0; r < info_.numRobots; ++r) {
        constraint(o * info_.numRobots + r) = (-getRobotLambda3d(input, info_, o, r).transpose() * robot[r].getB() -
                       getObstacleLambda3d(input, info_, o, r).transpose() * getObstacleB(parameters, o))(0) - 0.03;
      }
    }
    return constraint;//numObstacles*info_.numRobots x 1 
  }

   // Clone the Duality Lagrangian object
  DualityLagrangianAd* clone() const override { 
    return new DualityLagrangianAd(*this); 
  }
};




// 10(b)
class DualityVectorAd : public DualityBaseAd {
 public:
  // Constructor that initializes the Duality Vector with obstacles and information
  DualityVectorAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityVector", "/tmp/ocs2",
               true, false);
  }

  // Get the number of constraints
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { 
    return 3 * info_.numObstacles * info_.numRobots; 
  }

  // Calculate the constraint function for a given time, state, input, and parameters
  ad_vector_t constraintFunction(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {

    // Create a robot region based on the state and centroidal information
    auto robot = createRobotRegion3d(state, info_.centroidalInfo);// info_.numRobots x 1 vector

    ad_vector_t constraint(3 * info_.numObstacles * info_.numRobots);

    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (size_t r = 0; r < info_.numRobots; ++r) {
        constraint.segment(o * info_.numRobots * 3 + r * 3, 3) = robot[r].getA().transpose() * getRobotLambda3d(input, info_, o, r) +
                                                   getObstacleA(parameters, o).transpose() * getObstacleLambda3d(input, info_, o, r);
      }
    }

    return constraint;
  }

  DualityVectorAd* clone() const override { return new DualityVectorAd(*this); }
};




// 10d
class DualityPositiveAd : public DualityBaseAd {
 public:
  // Constructor that initializes the Duality Positive with obstacles and information
  DualityPositiveAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityPositive", "/tmp/ocs2",
               true, false);
  }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override {
    return info_.numObstacles * (6 + 6) * info_.numRobots;
  }

  ad_vector_t constraintFunction(ad_scalar_t /*time*/, const ad_vector_t& /*state*/, const ad_vector_t& input,
                                 const ad_vector_t& /*parameters*/) const override {
    return getLambda3D(input, info_);
  }

  DualityPositiveAd* clone() const override { return new DualityPositiveAd(*this); }
};




//10(c)
class DualityNormAd : public DualityBaseAd {
 public:

  DualityNormAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityNorm", "/tmp/ocs2", true,
               false);
  }
  
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override {
    return info_.numObstacles * info_.numRobots;
  }

  ad_vector_t constraintFunction(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    auto robot = createRobotRegion3d(state, info_.centroidalInfo);
    
    ad_vector_t constraint(info_.numRobots * info_.numObstacles);

    ad_vector_t lambda = getLambda3D(input, info_);

    for (size_t o = 0; o < info_.numObstacles; ++o)
      for (size_t r = 0; r < info_.numRobots; ++r){
        constraint(o * info_.numRobots + r) = -(getObstacleA(parameters, o).transpose() * getObstacleLambda3d(input, info_, o, r)).norm() + 1.;
      }

    return constraint;//numObstacles x 1
  }

  DualityNormAd* clone() const override { return new DualityNormAd(*this); }
};

}  // namespace cbf
