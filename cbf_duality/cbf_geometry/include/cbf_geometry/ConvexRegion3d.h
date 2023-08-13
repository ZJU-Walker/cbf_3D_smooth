//
// Created by Ke Wang on 2023/7/11.
//

#pragma once
#include <ocs2_core/Types.h>

namespace cbf {
using namespace ocs2;

template <typename SCALAR>
using vector_temp = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

template <typename SCALAR>
using matrix_temp = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

template <typename SCALAR>
class ConvexRegion3d {
 public:
  explicit ConvexRegion3d(size_t num_points) {
    this->a_ = matrix_temp<SCALAR>::Zero(6, 3);
    this->b_ = vector_temp<SCALAR>::Zero(6);
  }
  matrix_temp<SCALAR> getA() const { return a_; }
  vector_temp<SCALAR> getB() const { return b_; }

 protected:
  matrix_temp<SCALAR> a_;
  vector_temp<SCALAR> b_;
};

template <typename SCALAR>
class Rectangle3d : public ConvexRegion3d<SCALAR> {
 public:
  Rectangle3d(const vector_temp<SCALAR>& pose, const vector_temp<SCALAR>& size);
};

template <typename SCALAR>
class Cuboid3d : public ConvexRegion3d<SCALAR> {
 public:
  Cuboid3d(size_t num_points, const vector_temp<SCALAR>& points);
};

}  // namespace cbf
