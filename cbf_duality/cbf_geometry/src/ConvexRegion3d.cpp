#include "cbf_geometry/ConvexRegion3d.h"
#include <ocs2_core/automatic_differentiation/Types.h>
#include <qpOASES.hpp>
#include <Eigen/Dense>

namespace cbf {
using namespace ocs2;
using namespace Eigen;

template <typename SCALAR>
Rectangle3d<SCALAR>::Rectangle3d(const vector_temp<SCALAR>& pose, const vector_temp<SCALAR>& size) : ConvexRegion3d<SCALAR>(6) {
  matrix_temp<SCALAR> rot(3, 3);
  // Assuming Euler angles (roll:x, pitch:y, yaw:z) for rotation. 
  // Note: Be aware of gimbal lock and choose your rotation representation accordingly. 
  // size(0) is the length of the rectangle along the x-axis, size(1) is the length of the rectangle along the y-axis, size(2) is the length of the rectangle along the z-axis
  SCALAR roll = pose(5), pitch = pose(4), yaw = pose(3);
//   SCALAR roll = pose(3), pitch = pose(4), yaw = pose(5);
  matrix_temp<SCALAR> rotYaw(3, 3), rotPitch(3, 3), rotRoll(3, 3);
  rotYaw << SCALAR(cos(yaw)), SCALAR(-sin(yaw)), SCALAR(0),
            SCALAR(sin(yaw)), SCALAR(cos(yaw)), SCALAR(0),
            SCALAR(0), SCALAR(0), SCALAR(1);
  rotPitch << SCALAR(cos(pitch)), SCALAR(0), SCALAR(sin(pitch)),
              SCALAR(0), SCALAR(1), SCALAR(0),
              SCALAR(-sin(pitch)), SCALAR(0), SCALAR(cos(pitch));
  rotRoll << SCALAR(1), SCALAR(0), SCALAR(0),
             SCALAR(0), SCALAR(cos(roll)), SCALAR(-sin(roll)),
             SCALAR(0), SCALAR(sin(roll)), SCALAR(cos(roll));
  // rot = rotYaw * rotPitch * rotRoll;
  rot = rotRoll * rotPitch * rotYaw;
  
  this->a_ << SCALAR(1), SCALAR(0), SCALAR(0),
              SCALAR(-1), SCALAR(0), SCALAR(0), 
              SCALAR(0), SCALAR(1), SCALAR(0), 
              SCALAR(0), SCALAR(-1), SCALAR(0), 
              SCALAR(0), SCALAR(0), SCALAR(1), 
              SCALAR(0), SCALAR(0), SCALAR(-1); // a is 6x3
  this->a_ = this->a_ * rot;
  
  vector_temp<SCALAR> pos = rot * pose.head(3); // pose.head(3) represent x,y,z of the centroid point, rotate the centroid
  
  this->b_ << pos(0) + size(0) / 2, -(pos(0) - size(0) / 2), 
               pos(1) + size(1) / 2, -(pos(1) - size(1) / 2), 
               pos(2) + size(2) / 2, -(pos(2) - size(2) / 2); // b is 6x1
}


// explicit template instantiation
template class ConvexRegion3d<scalar_t>;
template class ConvexRegion3d<ad_scalar_t>;
template class Rectangle3d<scalar_t>;
template class Rectangle3d<ad_scalar_t>;


}  // namespace cbf
