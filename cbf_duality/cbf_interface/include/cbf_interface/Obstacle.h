//
// Created by Ke Wang on 2023/7/19.
// I need to define obstacles in 3D but rectangle method
//

#pragma once

#include "cbf_interface/FactoryAccessHelper.h"

#include <cbf_geometry/ConvexRegion3d.h>

namespace cbf {

class DualityObstacles {
 public:
  DualityObstacles(const DualityInfo& info) : info_(info) {
    for (size_t i = 0; i < info_.numObstacles; ++i) {
      // vector_temp<scalar_t> ZeroPose(6);
      // vector_temp<scalar_t> ZeroSize(3);
      // ZeroPose << 2, 2, 0.25, 0, 0, 0;
      // ZeroSize << 0.5, 0.5, 0.5;
      // cuboid_.push_back(Rectangle3d<scalar_t>(ZeroPose, ZeroSize));
      vector_temp<scalar_t> Obstalce_1_Pose(6);
      vector_temp<scalar_t> Obstalce_1_Size(3);
      Obstalce_1_Pose << 2, 0, 0.0, 0, 0, 0;
      Obstalce_1_Size << 0.3, 1.0, 0.04;
      cuboid_.push_back(Rectangle3d<scalar_t>(Obstalce_1_Pose, Obstalce_1_Size));

    }
  }

  void setObstacle3d(const vector_array2_t& Obsatcle3d) {
    cuboid_.clear();
    // for (const auto& i : Obsatcle3d) {
    //   cuboid_.push_back(Rectangle3d<scalar_t>(i[0], i[1]));
    // }

    // for (size_t i = 0; i < info_.numObstacles; ++i) {
      // vector_temp<scalar_t> ZeroPose(6);
      // vector_temp<scalar_t> ZeroSize(3);
      // ZeroPose << 2, 2, 0.25, 0, 0, 0;
      // ZeroSize << 0.5, 0.5, 0.5;
      // cuboid_.push_back(Rectangle3d<scalar_t>(ZeroPose, ZeroSize));
    // }

    vector_temp<scalar_t> Obstalce_1_Pose(6);
    vector_temp<scalar_t> Obstalce_1_Size(3);
    Obstalce_1_Pose << 2, 0, 0.0, 0, 0, 0;
    Obstalce_1_Size << 0.3, 1.0, 0.04;
    cuboid_.push_back(Rectangle3d<scalar_t>(Obstalce_1_Pose, Obstalce_1_Size));
    
  }

   // Get size of parameters
   // A and B
   // A : 3, B : 1
   // Each obsatcle has 6 constraints
  virtual size_t getParametersSize() const {
    return 4 * info_.numObstacles * 6;
  }

   // Get parameters
   // Pass A and B
  virtual vector_t getParameters() const {
    vector_t ret = vector_t::Zero(getParametersSize());
    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (size_t p = 0; p < 6; ++p) {
        ret.segment<3>(o * 4 * 6 + p * 3) = cuboid_[o].getA().row(p);
      }
      ret.segment(o * 4 * 6 + 6 * 3, 6) = cuboid_[o].getB();
    }
    return ret;
  }

   // Get region at index i
  Rectangle3d<scalar_t> getRegion3d(size_t i) const {
    return cuboid_[i];
  }

   // Get info
  DualityInfo getInfo() const { 
    return info_; 
  }

 protected:
  const DualityInfo& info_;
  std::vector<Rectangle3d<scalar_t>> cuboid_;
};


class CbfObstacles : public DualityObstacles {
 public:
  CbfObstacles(const DualityInfo& info) : DualityObstacles(info) {
    time_ = 0.;
    vector_array_t dists_(info_.numObstacles, vector_t::Zero(info_.numRobots));
  }

  void setDists(scalar_t time, const vector_array_t& dists) {
    time_ = time;
    dists_ = dists;
  }

  // Update parameters size (add time and dists)
  size_t getParametersSize() const override { 
    return DualityObstacles::getParametersSize() + 1 + info_.numObstacles * info_.numRobots; 
  }

  // Each robot has 9 boxs (1 body + 4 legs * 2), so each obstacle has 9 distances to robot
  vector_t getParameters() const override {
    vector_t ret = vector_t::Zero(4 * info_.numObstacles * 6 + 1 + info_.numObstacles * info_.numRobots);
    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (int p = 0; p < 6; ++p) {
        ret.segment<3>(o * 4 * 6 + p * 3) = cuboid_[o].getA().row(p);
      }
      ret.segment(o * 4 * 6 + 6 * 3, 6) = cuboid_[o].getB();
    }
    ret(4 * info_.numObstacles * 6) = time_;
    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (int p = 0; p < info_.numRobots; ++p) {
        ret.segment<1>(4 * info_.numObstacles * 6 + 1 + o * info_.numRobots + p) = dists_[o].row(p);
      }
    }
    return ret;
  }

 private:
  scalar_t time_;
  vector_array_t dists_;

};

}  // namespace cbf
