//
// Created by qiayuan on 2023/7/14.
//
#include "cbf_geometry/DistanceQp3d.h"
#include <iostream>
namespace cbf {
using namespace ocs2;

DistanceQp3d::DistanceQp3d(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1) {
  matrix_t i = matrix_t::Identity(3, 3);
  h_ << i, -i, -i, i;
  g_ = vector_t::Zero(6);

  vector_t b0 = region0.getB();
  vector_t b1 = region1.getB();

  c_ = matrix_t::Zero(b0.size() + b1.size(), 6);
  c_.block(0, 0, b0.size(), 3) = region0.getA();
  c_.block(b0.size(), 3, b1.size(), 3) = region1.getA();
  ub_ = vector_t::Zero(b0.size() + b1.size());
  ub_ << b0, b1;
      
  auto qpProblem = qpOASES::QProblem(6, ub_.size());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int n_wsr = 20;

  qpProblem.init(h_.data(), g_.data(), c_.data(), nullptr, nullptr, nullptr, ub_.data(), n_wsr);
  sol_ = vector_t(6);
  qpProblem.getPrimalSolution(sol_.data());
}

scalar_t DistanceQp3d::getDistance() {
  return sqrt(sol_.transpose() * h_ * sol_);
}

vector_t DistanceQp3d::getSolution() {
  return sol_;
}

Duality3d::Duality3d(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1)
    : numPointsRegion0_(region0.getB().size()),
      numPointsRegion1_(region1.getB().size()),
      numPoints_(numPointsRegion0_ + numPointsRegion1_) {
  // std::cout << "Robot: " << std::endl;
  // std::cout << "A: " << region0.getA() << std::endl;
  // std::cout << "b: " << region0.getB() << std::endl;
  // std::cout << "Obstacle: " << std::endl;
  // std::cout << "A: " << region1.getA() << std::endl;
  // std::cout << "b: " << region1.getB() << std::endl;

  // std::cout << "********************************************************" << std::endl;
  // std::cout << "DistanceQp3d.cpp Duality3d contructor begins" << std::endl;
  // std::cout << "********************************************************" << std::endl;

  // std::cout << "========================================================" << std::endl;
  // std::cout << "DistanceQp3d.cpp setupCost begins" << std::endl;
  // std::cout << "========================================================" << std::endl;

  setupCost(region0, region1);

  // std::cout << "========================================================" << std::endl;
  // std::cout << "DistanceQp3d.cpp setupCost ends" << std::endl;
  // std::cout << "========================================================" << std::endl;

  // std::cout << "========================================================" << std::endl;
  // std::cout << "DistanceQp3d.cpp setupConstraints begins" << std::endl;
  // std::cout << "========================================================" << std::endl;

  setupConstraints(region0, region1);

  // std::cout << "========================================================" << std::endl;
  // std::cout << "DistanceQp3d.cpp setupConstraints ends" << std::endl;
  // std::cout << "========================================================" << std::endl;

  qpProblem_ = std::make_shared<qpOASES::QProblem>(numPoints_, c_.rows());
  qpOASES::Options options;
  options.setToReliable();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem_->setOptions(options);
  int n_wsr = 50;
  qpProblem_->init(h_.data(), g_.data(), c_.data(), l_.data(), nullptr, lb_.data(), ub_.data(), n_wsr);

  // std::cout << "********************************************************" << std::endl;
  // std::cout << "DistanceQp3d.cpp Duality3d contructor ends" << std::endl;
  // std::cout << "********************************************************" << std::endl;

}

scalar_t Duality3d::getDistance() {
  return sqrt(-qpProblem_->getObjVal());
}

vector_t Duality3d::getSolution() {
  vector_t sol = vector_t(h_.rows());
  qpProblem_->getPrimalSolution(sol.data());
  return sol;
}

void Duality3d::setupCost(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1) {
  h_ = matrix_t(numPoints_, numPoints_);
  h_ << region0.getA() * region0.getA().transpose(), matrix_t::Zero(numPointsRegion0_, numPointsRegion1_),
      matrix_t::Zero(numPointsRegion1_, numPoints_);
  h_ *= 0.5;
  g_ = vector_t(numPoints_);
  g_ << region0.getB(), region1.getB();
}

void Duality3d::setupConstraints(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1) {
  c_ = matrix_t(6, numPoints_);
  c_.block(0, 0, 3, numPointsRegion0_) = region0.getA().transpose();
  c_.block(0, numPointsRegion0_, 3, numPointsRegion1_) = region1.getA().transpose();
  // c_ << region0.getA().transpose(), region1.getA().transpose();
  c_.block(3, 0, 3, numPoints_) = -c_.block(0, 0, 3, numPoints_);

  l_ = vector_t::Zero(numPoints_);
  ub_ = vector_t::Zero(6);
  lb_ = vector_t::Zero(6);
}

}  // namespace cbf
