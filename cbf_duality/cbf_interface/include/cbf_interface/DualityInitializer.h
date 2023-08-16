//
// Created by Ke Wang on 2023/7/18.
//

#pragma once

#include <ocs2_legged_robot/common/utils.h>

#include <cbf_geometry/DistanceQp3d.h>

namespace cbf {
using namespace ocs2;
using namespace legged_robot;

class DualityInitializer : public Initializer {
 public:
  DualityInitializer(DualityInfo info, const SwitchedModelReferenceManager& referenceManager, const DualityObstacles& obstacles);
  ~DualityInitializer() override = default;
  DualityInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  DualityInitializer(const DualityInitializer& other) = default;
  const DualityInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const DualityObstacles& obstacles_;

  scalar_t lastTime_{};
  vector_t lastState_;
  std::vector<std::shared_ptr<Duality3d>> qp_;// change to 3d
};

DualityInitializer::DualityInitializer(DualityInfo info, const SwitchedModelReferenceManager& referenceManager,
                                       const DualityObstacles& obstacles)
    : info_(std::move(info)), referenceManagerPtr_(&referenceManager), obstacles_(obstacles), qp_(info_.numObstacles * info_.numRobots) {
  lastState_ = vector_t::Zero(info_.centroidalInfo.stateDim);
}

DualityInitializer* DualityInitializer::clone() const {
  return new DualityInitializer(*this);
}

void DualityInitializer::compute(scalar_t time, const vector_t& state, scalar_t /*nextTime*/, vector_t& input, vector_t& nextState) {
  nextState = state;

  const auto contact_flags = referenceManagerPtr_->getContactFlags(time);
  input = weightCompensatingInput(info_.centroidalInfo, contact_flags);
  auto robot = createRobotRegion3d(state, info_.centroidalInfo);// need to change to 3d
  bool need_update = false;
  for (size_t o = 0; o < info_.numObstacles; o++) {
    for (size_t r = 0; r < info_.numRobots; r++) {
      if (qp_[o * info_.numRobots + r] == nullptr) {
        need_update = true;
      }
    }
  }
  if (lastTime_ != time && !lastState_.isApprox(state)) {
    need_update = true;
  }
  if (need_update) {
    for (size_t o = 0; o < info_.numObstacles; o++) {
      for (size_t r = 0; r < info_.numRobots; r++) {
        auto obstacle = obstacles_.getRegion3d(o);
        qp_[o * info_.numRobots + r] = std::make_shared<Duality3d>(robot[r], obstacle);// change to 3d
      }
    }
  }

  getLambda3D(input, info_).setZero();// need to change to 3d, lambda is 12 x 1
  // qp_[o] is solution of dual problem, sol_ = [lambda_R; lambda_O] ((n_R+n_O) x 1)
  // 3D, for robot and obstacle, only consider cuboid, means dimension of a is 6x3, b is 6x1, lambda is 6x1, num_points = 6 (actually is num_face, = 6)

  for (size_t o = 0; o < info_.numObstacles; o++) {
    for (size_t r =0; r < info_.numRobots; r++) {
      double dist = qp_[o * info_.numRobots + r]->getDistance();
      // std::cout << "[DualityInitializer.h] dist: " << dist << std::endl;
      if (!isnan(dist) && dist > 1e-3) {
        vector_t lambda = (0.5 / dist) * qp_[o * info_.numRobots + r]->getSolution();// TODO
        // std::cout << "[DualityInitializer.h] lambda:\n " << lambda << std::endl;
        // vector_t lambda = qp_[o * info_.numRobots + r]->getSolution();
        getRobotLambda3d(input, info_, o, r) = lambda.head(6);
        getObstacleLambda3d(input, info_, o, r) = lambda.tail(6);// original is info_.numPoints, but only consider cuboid, so num_points(faces) = 6
      }
    }
  }
}

}  // namespace cbf
