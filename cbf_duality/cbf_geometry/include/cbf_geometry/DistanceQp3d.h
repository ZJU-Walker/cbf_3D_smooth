//
// Created by qiayuan on 2022/8/8.
//
#pragma once
#include <memory>

#include <ocs2_core/Types.h>
#include <qpOASES.hpp>

#include "cbf_geometry/ConvexRegion3d.h"

namespace cbf {

// calculates the distance between two convex regions in a 3D space
class DistanceQp3d {
 public:
  DistanceQp3d(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1);
  scalar_t getDistance();
  vector_t getSolution();

 private:
  Eigen::Matrix<scalar_t, 6, 6, Eigen::RowMajor> h_;
  Eigen::Matrix<scalar_t, Eigen::Dynamic, 6, Eigen::RowMajor> c_;
  vector_t g_, ub_;

  vector_t sol_;
};

class Duality3d {
 public:
  Duality3d(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1);
  scalar_t getDistance();
  vector_t getSolution();

 private:
  void setupCost(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1);
  void setupConstraints(const ConvexRegion3d<scalar_t>& region0, ConvexRegion3d<scalar_t>& region1);

  std::shared_ptr<qpOASES::QProblem> qpProblem_;

  size_t numPointsRegion0_, numPointsRegion1_, numPoints_;

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_;
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic, Eigen::RowMajor> c_;
  vector_t g_, ub_, lb_, l_;
};

}  // namespace cbf
