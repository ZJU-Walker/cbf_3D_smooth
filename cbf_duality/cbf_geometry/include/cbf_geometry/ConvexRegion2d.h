//
// Created by qiayuan on 2022/7/25.
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
class ConvexRegion2d {
//here define ConvexRegion in Linear constraint Ax<=B, the class ConvexRegion2d contains 2 method to define the constraint, the Rectangle and Vertex method
//define 2 function to get A and B to represent the convex area
 public:
  explicit ConvexRegion2d(size_t num_points) {
    this->a_ = matrix_temp<SCALAR>::Zero(num_points, 2);//The matrix's dimensions are num_points x 2 and are initialized to zero
    this->b_ = vector_temp<SCALAR>::Zero(num_points);//The vector's dimensions are num_points x 1 and are initialized to zero
  }
  matrix_temp<SCALAR> getA() const { return a_; }
  vector_temp<SCALAR> getB() const { return b_; }

 protected:
  matrix_temp<SCALAR> a_;
  vector_temp<SCALAR> b_;
};
//rectangle method:
//for a point (x,y) and a rectangle left bottom point(x0,y0) and right upper point(x1,y1)
// if x0<x<x1; y0<y<y1, then it is in this area
template <typename SCALAR>
class Rectangle2d : public ConvexRegion2d<SCALAR> {
 public:
  Rectangle2d(const vector_temp<SCALAR>& pose, const vector_temp<SCALAR>& size);
};
//vertex method:
//use cross product to define a convex area
//take 3 points each time, if the are of the triangle (calculated by cross product) is positive (for all points)
//it is a convex area  
template <typename SCALAR>
class Vertex2d : public ConvexRegion2d<SCALAR> {
 public:
  Vertex2d(size_t num_points, const vector_temp<SCALAR>& points);
};

}  // namespace cbf
