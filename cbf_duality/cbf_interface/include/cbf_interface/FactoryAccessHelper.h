//
// Created by qiayuan on 2022/7/22.
// Edited by Ke Wang on 2023/7/18.
// Add follow functions:
// 1. getLambda3D
// 2. getObstacleLambda3d
// 3. getRobotLambda3d
// 4. createRobotRegion3d
// Edited by Ke Wang on 2023/7/20.
// Change getlambda function, consider a robot in 3d contains 9 boxes
//

#pragma once

#include <cbf_geometry/ConvexRegion3d.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <cmath>

namespace cbf {
using namespace ocs2;

struct DualityInfo {
  CentroidalModelInfo centroidalInfo;
  size_t numObstacles{};
  size_t numPoints{};
  size_t numRobots{};
};

DualityInfo createDualityInfo(const CentroidalModelInfo& info, size_t numObstacles, size_t numPoints, size_t numRobots);


template <typename SCALAR>
vector_temp<SCALAR> eulerAnglesFromRotationMatrix(matrix_temp<SCALAR> R) {
    vector_temp<SCALAR> angles(3);
    angles[2] = atan2(R(1, 0), R(0, 0)); // Roll (X)
    angles[1] = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))); // Pitch (Y)
    angles[0] = atan2(R(2, 1), R(2, 2)); // Yaw (Z)
    return angles;
}

// convert point in thigh frame to world frame
template <typename SCALAR>
void thighToWorld(const vector_temp<SCALAR>& base_origin, const vector_temp<SCALAR>& base_pose,
                  const vector_temp<SCALAR>& thigh_origin, const vector_temp<SCALAR>& thigh_pose,
                  const vector_temp<SCALAR>& point_in_thigh, vector_temp<SCALAR>& point_in_world, vector_temp<SCALAR>& pose_in_world) {
  matrix_temp<SCALAR> R_thigh_to_base(3, 3);
  SCALAR roll_thigh_to_base = thigh_pose[2];
  SCALAR pitch_thigh_to_base = thigh_pose[1];
  SCALAR yaw_thigh_to_base = thigh_pose[0];
  R_thigh_to_base << cos(yaw_thigh_to_base) * cos(pitch_thigh_to_base), cos(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * sin(roll_thigh_to_base) - sin(yaw_thigh_to_base) * cos(roll_thigh_to_base), cos(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * cos(roll_thigh_to_base) + sin(yaw_thigh_to_base) * sin(roll_thigh_to_base),
                     sin(yaw_thigh_to_base) * cos(pitch_thigh_to_base), sin(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * sin(roll_thigh_to_base) + cos(yaw_thigh_to_base) * cos(roll_thigh_to_base), sin(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * cos(roll_thigh_to_base) - cos(yaw_thigh_to_base) * sin(roll_thigh_to_base),
                     -sin(pitch_thigh_to_base), cos(pitch_thigh_to_base) * sin(roll_thigh_to_base), cos(pitch_thigh_to_base) * cos(roll_thigh_to_base);
  vector_temp<SCALAR> point_thigh(3);
  point_thigh << point_in_thigh[0], point_in_thigh[1], point_in_thigh[2];
  vector_temp<SCALAR> point_base = R_thigh_to_base * point_thigh + thigh_origin;

  matrix_temp<SCALAR> R_base_to_world(3,3);
  SCALAR roll_base_to_world = base_pose[2];
  SCALAR pitch_base_to_world = base_pose[1];
  SCALAR yaw_base_to_world = base_pose[0];
  R_base_to_world << cos(yaw_base_to_world) * cos(pitch_base_to_world), cos(yaw_base_to_world) * sin(pitch_base_to_world) * sin(roll_base_to_world) - sin(yaw_base_to_world) * cos(roll_base_to_world), cos(yaw_base_to_world) * sin(pitch_base_to_world) * cos(roll_base_to_world) + sin(yaw_base_to_world) * sin(roll_base_to_world),
                     sin(yaw_base_to_world) * cos(pitch_base_to_world), sin(yaw_base_to_world) * sin(pitch_base_to_world) * sin(roll_base_to_world) + cos(yaw_base_to_world) * cos(roll_base_to_world), sin(yaw_base_to_world) * sin(pitch_base_to_world) * cos(roll_base_to_world) - cos(yaw_base_to_world) * sin(roll_base_to_world),
                     -sin(pitch_base_to_world), cos(pitch_base_to_world) * sin(roll_base_to_world), cos(pitch_base_to_world) * cos(roll_base_to_world);
  vector_temp<SCALAR> point_world = R_base_to_world * point_base + base_origin;
  matrix_temp<SCALAR> R_thigh_to_world(3,3);
  R_thigh_to_world = R_base_to_world * R_thigh_to_base;
  
  vector_temp<SCALAR> euler_angles = eulerAnglesFromRotationMatrix(R_thigh_to_world); // ZYX顺序
  // vector_temp <SCALAR> euler_angles(3);
  // euler_angles << SCALAR(0), SCALAR(0), SCALAR(0);
  // std::cout << "size" << R_thigh_to_world.size() << std::endl;

  point_in_world = point_world;
  pose_in_world = euler_angles;
}; 

// convert point in calf frame to world frame
template <typename SCALAR>
void calfToWorld(const vector_temp<SCALAR>& base_origin, const vector_temp<SCALAR>& base_pose,
                 const vector_temp<SCALAR>& thigh_origin, const vector_temp<SCALAR>& thigh_pose,
                 const vector_temp<SCALAR>& calf_origin, const vector_temp<SCALAR>& calf_pose,
                 const vector_temp<SCALAR>& point_in_calf, vector_temp<SCALAR>& point_in_world, vector_temp<SCALAR>& pose_in_world) {
  matrix_temp<SCALAR> R_thigh_to_base(3, 3);
  SCALAR roll_thigh_to_base = thigh_pose[2];
  SCALAR pitch_thigh_to_base = thigh_pose[1];
  SCALAR yaw_thigh_to_base = thigh_pose[0];
  R_thigh_to_base << cos(yaw_thigh_to_base) * cos(pitch_thigh_to_base), cos(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * sin(roll_thigh_to_base) - sin(yaw_thigh_to_base) * cos(roll_thigh_to_base), cos(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * cos(roll_thigh_to_base) + sin(yaw_thigh_to_base) * sin(roll_thigh_to_base),
                     sin(yaw_thigh_to_base) * cos(pitch_thigh_to_base), sin(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * sin(roll_thigh_to_base) + cos(yaw_thigh_to_base) * cos(roll_thigh_to_base), sin(yaw_thigh_to_base) * sin(pitch_thigh_to_base) * cos(roll_thigh_to_base) - cos(yaw_thigh_to_base) * sin(roll_thigh_to_base),
                     -sin(pitch_thigh_to_base), cos(pitch_thigh_to_base) * sin(roll_thigh_to_base), cos(pitch_thigh_to_base) * cos(roll_thigh_to_base);
    
  matrix_temp<SCALAR> R_calf_to_thigh(3, 3);
  SCALAR roll_calf_to_thigh = calf_pose[2];
  SCALAR pitch_calf_to_thigh = calf_pose[1];
  SCALAR yaw_calf_to_thigh = calf_pose[0];
  R_calf_to_thigh << cos(yaw_calf_to_thigh) * cos(pitch_calf_to_thigh), cos(yaw_calf_to_thigh) * sin(pitch_calf_to_thigh) * sin(roll_calf_to_thigh) - sin(yaw_calf_to_thigh) * cos(roll_calf_to_thigh), cos(yaw_calf_to_thigh) * sin(pitch_calf_to_thigh) * cos(roll_calf_to_thigh) + sin(yaw_calf_to_thigh) * sin(roll_calf_to_thigh),
                     sin(yaw_calf_to_thigh) * cos(pitch_calf_to_thigh), sin(yaw_calf_to_thigh) * sin(pitch_calf_to_thigh) * sin(roll_calf_to_thigh) + cos(yaw_calf_to_thigh) * cos(roll_calf_to_thigh), sin(yaw_calf_to_thigh) * sin(pitch_calf_to_thigh) * cos(roll_calf_to_thigh) - cos(yaw_calf_to_thigh) * sin(roll_calf_to_thigh),
                     -sin(pitch_calf_to_thigh), cos(pitch_calf_to_thigh) * sin(roll_calf_to_thigh), cos(pitch_calf_to_thigh) * cos(roll_calf_to_thigh);
  
  vector_temp<SCALAR> point_calf(3);
  point_calf << point_in_calf[0], point_in_calf[1], point_in_calf[2];
  vector_temp<SCALAR> point_thigh = R_calf_to_thigh * point_calf + calf_origin;
  vector_temp<SCALAR> point_base = R_thigh_to_base * point_thigh + thigh_origin;
  
  matrix_temp<SCALAR> R_base_to_world(3,3);
  SCALAR roll_base_to_world = base_pose[2];
  SCALAR pitch_base_to_world = base_pose[1];
  SCALAR yaw_base_to_world = base_pose[0];
  R_base_to_world << cos(yaw_base_to_world) * cos(pitch_base_to_world), cos(yaw_base_to_world) * sin(pitch_base_to_world) * sin(roll_base_to_world) - sin(yaw_base_to_world) * cos(roll_base_to_world), cos(yaw_base_to_world) * sin(pitch_base_to_world) * cos(roll_base_to_world) + sin(yaw_base_to_world) * sin(roll_base_to_world),
                     sin(yaw_base_to_world) * cos(pitch_base_to_world), sin(yaw_base_to_world) * sin(pitch_base_to_world) * sin(roll_base_to_world) + cos(yaw_base_to_world) * cos(roll_base_to_world), sin(yaw_base_to_world) * sin(pitch_base_to_world) * cos(roll_base_to_world) - cos(yaw_base_to_world) * sin(roll_base_to_world),
                     -sin(pitch_base_to_world), cos(pitch_base_to_world) * sin(roll_base_to_world), cos(pitch_base_to_world) * cos(roll_base_to_world);
  
  vector_temp<SCALAR> point_world = R_base_to_world * point_base + base_origin;
  matrix_temp<SCALAR> R_calf_to_world(3,3);
  R_calf_to_world = R_base_to_world * R_thigh_to_base * R_calf_to_thigh;

  vector_temp<SCALAR> euler_angles = eulerAnglesFromRotationMatrix(R_calf_to_world); // ZYX顺序

  point_in_world = point_world;
  pose_in_world = euler_angles;
};

template <typename Derived>
Eigen::Block<Derived, -1, 1> getLambda2D(Eigen::MatrixBase<Derived>& input, const DualityInfo& info) {
  const size_t startRow =
      info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts + 6 * info.centroidalInfo.numSixDofContacts;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.numObstacles * 4 + info.numObstacles * info.numPoints, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getLambda2D(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info) {
  const size_t startRow =
      info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts + 6 * info.centroidalInfo.numSixDofContacts;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.numObstacles * 4 + info.numObstacles * info.numPoints, 1);
}

// Add get lambda for 3d
template <typename Derived>
Eigen::Block<Derived, -1, 1> getLambda3D(Eigen::MatrixBase<Derived>& input, const DualityInfo& info) {
  const size_t startRow =
      info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts + 6 * info.centroidalInfo.numSixDofContacts;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.numRobots * (info.numObstacles * 6 + info.numObstacles * 6), 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getLambda3D(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info) {
  const size_t startRow =
      info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts + 6 * info.centroidalInfo.numSixDofContacts;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.numRobots * (info.numObstacles * 6 + info.numObstacles * 6), 1);
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getObstacleLambda(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints) + 4;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.numPoints, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getObstacleLambda(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints) + 4;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.numPoints, 1);
}

// Add getObstacleLambda3d
template <typename Derived>
Eigen::Block<Derived, -1, 1> getObstacleLambda3d(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o, size_t r) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + info.numRobots * o * 12 + r * 12 + 6;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, 6, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getObstacleLambda3d(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o, size_t r) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + info.numRobots * o * 12 + r * 12 + 6;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, 6, 1);
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getRobotLambda(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints);
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, 4, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getRobotLambda(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  //  const size_t startRow = o * (4 + info.numPoints);
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints);
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, 4, 1);
}

// Add getRobotLambda3d
template <typename Derived>
Eigen::Block<Derived, -1, 1> getRobotLambda3d(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o, size_t r) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + info.numRobots * o * 12 + r * 12;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, 6, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getRobotLambda3d(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o, size_t r) {
  //  const size_t startRow = o * (4 + info.numPoints);
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + info.numRobots * o * 12 + r * 12;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, 6, 1);
}

// Add createRobotRegion3d, here use rectangle3d method
template <typename SCALAR>
Rectangle3d<SCALAR> createRobotRegion3d_body(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info) {
  vector_temp<SCALAR> basePose = centroidal_model::getBasePose(state, info);
  vector_temp<SCALAR> size(3);
  size << SCALAR(0.59), SCALAR(0.32), SCALAR(0.15);
  return Rectangle3d<SCALAR>(basePose, size);
}

// create thigh (4)
template <typename SCALAR>
Rectangle3d<SCALAR> createRobotRegion3d_thigh(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info, size_t i) {
  vector_temp<SCALAR> basePose = centroidal_model::getBasePose(state, info);
  vector_temp<SCALAR> base_origin(3);
  vector_temp<SCALAR> base_pose(3);
  vector_temp<SCALAR> thigh1_origin(3);
  vector_temp<SCALAR> thigh2_origin(3);
  vector_temp<SCALAR> thigh3_origin(3);
  vector_temp<SCALAR> thigh4_origin(3);
  vector_temp<SCALAR> thigh1_pose(3);
  vector_temp<SCALAR> thigh2_pose(3);
  vector_temp<SCALAR> thigh3_pose(3);
  vector_temp<SCALAR> thigh4_pose(3);
  vector_temp<SCALAR> thigh_center_inthighframe(3);// center of thigh in thigh frame
  vector_temp<SCALAR> thigh_center_inworldframe(3);// center of thigh in world frame
  vector_temp<SCALAR> thigh_pose_inworldframe(3);// thigh pose in world frame

  base_origin << basePose(0), basePose(1), basePose(2);
  base_pose << basePose(3), basePose(4), basePose(5);// ZYX
  thigh1_origin << SCALAR(0.15), SCALAR(0.10), SCALAR(-0.04);// TODO: may need to change according to body shape
  thigh2_origin << SCALAR(-0.15), SCALAR(0.10), SCALAR(-0.04);
  thigh3_origin << SCALAR(-0.15), SCALAR(-0.10), SCALAR(-0.04);
  thigh4_origin << SCALAR(0.15), SCALAR(-0.10), SCALAR(-0.04);
  // Leg Joint Positions: [LF, LH, RF, RH] LF_HAA LF_HFE LF_KFE
  vector_temp<SCALAR> jointAngle = centroidal_model::getJointAngles(state, info);// 12 x 1
  // TODO: here test
  // vector_temp<SCALAR> jointAngle(12);
  // jointAngle << SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0);
  thigh1_pose << SCALAR(0), jointAngle(1), jointAngle(0);//LF
  thigh2_pose << SCALAR(0), jointAngle(4), jointAngle(3);//LH
  thigh3_pose << SCALAR(0), jointAngle(7), jointAngle(6);//RF
  thigh4_pose << SCALAR(0), jointAngle(10), jointAngle(9);//RH
  // Initialize center of thigh to zero
  thigh_center_inthighframe << SCALAR(0), SCALAR(0), SCALAR(-0.1);

  // These 2 paras are to be changed by function thighToWorld
  thigh_center_inworldframe << SCALAR(0), SCALAR(0), SCALAR(0);
  thigh_pose_inworldframe << SCALAR(0), SCALAR(0), SCALAR(0);

  if (i == 1) {
    thighToWorld(base_origin, base_pose, thigh1_origin, thigh1_pose, thigh_center_inthighframe, thigh_center_inworldframe, thigh_pose_inworldframe);
  }
  if (i == 2) {
    thighToWorld(base_origin, base_pose, thigh2_origin, thigh2_pose, thigh_center_inthighframe, thigh_center_inworldframe, thigh_pose_inworldframe);
  }
  if (i == 3) {
    thighToWorld(base_origin, base_pose, thigh3_origin, thigh3_pose, thigh_center_inthighframe, thigh_center_inworldframe, thigh_pose_inworldframe);
  }
  if (i == 4) {
    thighToWorld(base_origin, base_pose, thigh4_origin, thigh4_pose, thigh_center_inthighframe, thigh_center_inworldframe, thigh_pose_inworldframe);
  }

  vector_temp<SCALAR> thigh_center(6);
  // TODO: test here
  // thigh_center << SCALAR(0), SCALAR(0), SCALAR(0), SCALAR(0), SCALAR(0), SCALAR(0);
  thigh_center << thigh_center_inworldframe(0), thigh_center_inworldframe(1), thigh_center_inworldframe(2), thigh_pose_inworldframe(0), thigh_pose_inworldframe(1), thigh_pose_inworldframe(2);
  vector_temp<SCALAR> size(3);
  size << SCALAR(0.04), SCALAR(0.04), SCALAR(0.247);
  return Rectangle3d<SCALAR>(thigh_center, size);
}

// create calf (4)
template <typename SCALAR>
Rectangle3d<SCALAR> createRobotRegion3d_calf(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info, size_t i) {
  vector_temp<SCALAR> basePose = centroidal_model::getBasePose(state, info);
  vector_temp<SCALAR> jointAngle = centroidal_model::getJointAngles(state, info);// 12 x 1
  // TODO: test here
  // vector_temp<SCALAR> jointAngle(12);
  // jointAngle << SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0);
  vector_temp<SCALAR> base_origin(3);
  vector_temp<SCALAR> base_pose(3);
  vector_temp<SCALAR> thigh1_origin(3);
  vector_temp<SCALAR> thigh2_origin(3);
  vector_temp<SCALAR> thigh3_origin(3);
  vector_temp<SCALAR> thigh4_origin(3);
  vector_temp<SCALAR> thigh1_pose(3);
  vector_temp<SCALAR> thigh2_pose(3);
  vector_temp<SCALAR> thigh3_pose(3);
  vector_temp<SCALAR> thigh4_pose(3);
  vector_temp<SCALAR> calf1_origin(3);
  vector_temp<SCALAR> calf2_origin(3);
  vector_temp<SCALAR> calf3_origin(3);
  vector_temp<SCALAR> calf4_origin(3);
  vector_temp<SCALAR> calf1_pose(3);
  vector_temp<SCALAR> calf2_pose(3);
  vector_temp<SCALAR> calf3_pose(3);
  vector_temp<SCALAR> calf4_pose(3);
  vector_temp<SCALAR> calf_center_incalfframe(3);// center of calf in calf frame
  vector_temp<SCALAR> calf_center_inworldframe(3);// center of calf in world frame
  vector_temp<SCALAR> calf_pose_inworldframe(3);// calf pose in world frame

  base_origin << basePose(0), basePose(1), basePose(2);
  base_pose << basePose(3), basePose(4), basePose(5);// ZYX

  thigh1_origin << SCALAR(0.15), SCALAR(0.10), SCALAR(-0.04);// TODO: may need to change according to body shape
  thigh2_origin << SCALAR(-0.15), SCALAR(0.10), SCALAR(-0.04);
  thigh3_origin << SCALAR(-0.15), SCALAR(-0.10), SCALAR(-0.04);
  thigh4_origin << SCALAR(0.15), SCALAR(-0.10), SCALAR(-0.04);

  calf1_origin  << SCALAR(0), SCALAR(0), SCALAR(-0.23);
  calf2_origin  << SCALAR(0), SCALAR(0), SCALAR(-0.23);
  calf3_origin  << SCALAR(0), SCALAR(0), SCALAR(-0.23);
  calf4_origin  << SCALAR(0), SCALAR(0), SCALAR(-0.23);
  // assuming that angle is stored: hip1_1, hip1_2, hip1_3, hip1_4, hip2_1, hip2_2, hip2_3, hip2_4. knee1, knee2, knee3, knee4
  thigh1_pose << SCALAR(0), jointAngle(1), jointAngle(0);//LF
  thigh2_pose << SCALAR(0), jointAngle(4), jointAngle(3);//LH
  thigh3_pose << SCALAR(0), jointAngle(7), jointAngle(6);//RF
  thigh4_pose << SCALAR(0), jointAngle(10), jointAngle(9);//RH

  calf1_pose << SCALAR(0), jointAngle(2), SCALAR(0);//LF
  calf2_pose << SCALAR(0), jointAngle(5), SCALAR(0);//LH
  calf3_pose << SCALAR(0), jointAngle(8), SCALAR(0);//RF
  calf4_pose << SCALAR(0), jointAngle(11), SCALAR(0);//RH

  calf_center_incalfframe << SCALAR(0), SCALAR(0), SCALAR(-0.1);

  if (i == 1) {
    calfToWorld(base_origin, base_pose, thigh1_origin, thigh1_pose, calf1_origin, calf1_pose, calf_center_incalfframe, calf_center_inworldframe, calf_pose_inworldframe);
  }
  if (i == 2) {
    calfToWorld(base_origin, base_pose, thigh2_origin, thigh2_pose, calf2_origin, calf2_pose, calf_center_incalfframe, calf_center_inworldframe, calf_pose_inworldframe);
  }
  if (i == 3) {
    calfToWorld(base_origin, base_pose, thigh3_origin, thigh3_pose, calf3_origin, calf3_pose, calf_center_incalfframe, calf_center_inworldframe, calf_pose_inworldframe);
  }
  if (i == 4) {
    calfToWorld(base_origin, base_pose, thigh4_origin, thigh4_pose, calf4_origin, calf4_pose, calf_center_incalfframe, calf_center_inworldframe, calf_pose_inworldframe);
  }

  vector_temp<SCALAR> calf_center(6);
  // TODO: test here
  // calf_center << SCALAR(0), SCALAR(0), SCALAR(0), SCALAR(0), SCALAR(0), SCALAR(0);
  calf_center << calf_center_inworldframe(0), calf_center_inworldframe(1), calf_center_inworldframe(2), calf_pose_inworldframe(0), calf_pose_inworldframe(1), calf_pose_inworldframe(2);
  vector_temp<SCALAR> size(3);
  size << SCALAR(0.04), SCALAR(0.04), SCALAR(0.247);
  return Rectangle3d<SCALAR>(calf_center, size);
}

// create robot region 3d (9 x 1)
// I want to create a 9 x 1 vector, each element is a rectangle3d
// 1. body (1)
// 2. thigh (4)
// 3. calf (4)
template <typename SCALAR>
std::vector<Rectangle3d<SCALAR>> createRobotRegion3d (const vector_temp<SCALAR>& state, const CentroidalModelInfo& info) {
  std::vector<Rectangle3d<SCALAR>> ret;
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  // ret.push_back(createRobotRegion3d_body(state, info));
  ret.push_back(createRobotRegion3d_thigh(state, info, 1));
  ret.push_back(createRobotRegion3d_thigh(state, info, 2));
  ret.push_back(createRobotRegion3d_thigh(state, info, 3));
  ret.push_back(createRobotRegion3d_thigh(state, info, 4));
  ret.push_back(createRobotRegion3d_calf(state, info, 1));
  ret.push_back(createRobotRegion3d_calf(state, info, 2));
  ret.push_back(createRobotRegion3d_calf(state, info, 3));
  ret.push_back(createRobotRegion3d_calf(state, info, 4));
  return ret;
}

}  // namespace cbf
