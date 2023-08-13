//
// Created by qiayuan on 2022/7/22.
//

#include "cbf_interface/FactoryAccessHelper.h"

namespace cbf {
CentroidalModelInfo createCbfModelInfo(const CentroidalModelInfo& info) {
  CentroidalModelInfo cbfInfo = info;
  cbfInfo.inputDim += 1;
  return cbfInfo;
}

// 3d
DualityInfo createDualityInfo(const CentroidalModelInfo& info, size_t numObstacles, size_t numPoints, size_t numRobots) {
  DualityInfo polyInfo;
  polyInfo.centroidalInfo = info;
  polyInfo.numObstacles = numObstacles;
  polyInfo.numPoints = 6;
  polyInfo.numRobots = numRobots;
  polyInfo.centroidalInfo.inputDim += numRobots * (numObstacles * 6 + numObstacles * 6);

  return polyInfo;
}

// 将thigh坐标系中的点T转换到world坐标系中
// base_origin - base坐标系原点在world坐标系中的坐标
// base_pose - base坐标系的位姿
// thigh_origin - thigh坐标系原点在base坐标系中的坐标
// thigh_pose - thigh坐标系的位姿
// point_in_thigh - thigh坐标系中的点T的坐标
// point_in_world - T在world坐标系中的坐标
// pose_in_world - thigh在world坐标系中的位姿
// void thighToWorld(const vector_temp<double>& base_origin, const vector_temp<double>& base_pose,
//                   const vector_temp<double>& thigh_origin, const vector_temp<double>& thigh_pose,
//                   const vector_temp<double>& point_in_thigh, vector_temp<double>& point_in_world, vector_temp<double>& pose_in_world) {
//     Eigen::Matrix3d R_thigh_to_base;
//     R_thigh_to_base = Eigen::AngleAxisd(thigh_pose[2], Eigen::Vector3d::UnitX()) *
//                       Eigen::AngleAxisd(thigh_pose[1], Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(thigh_pose[0], Eigen::Vector3d::UnitZ());

//     Eigen::Vector3d point_thigh(point_in_thigh[0], point_in_thigh[1], point_in_thigh[2]);
//     Eigen::Vector3d point_base = R_thigh_to_base * point_thigh + Eigen::Vector3d(thigh_origin[0], thigh_origin[1], thigh_origin[2]);

//     Eigen::Matrix3d R_base_to_world;
//     R_base_to_world = Eigen::AngleAxisd(base_pose[2], Eigen::Vector3d::UnitX()) *
//                       Eigen::AngleAxisd(base_pose[1], Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(base_pose[0], Eigen::Vector3d::UnitZ());

//     Eigen::Vector3d point_world = R_base_to_world * point_base + Eigen::Vector3d(base_origin[0], base_origin[1], base_origin[2]);

//     Eigen::Matrix3d R_thigh_to_world = R_base_to_world * R_thigh_to_base;
    // Eigen::Vector3d euler_angles = R_thigh_to_world.eulerAngles(2, 1, 0); // ZYX顺序

//     point_in_world[0] = point_world[0];
//     point_in_world[1] = point_world[1];
//     point_in_world[2] = point_world[2];

//     pose_in_world[0] = euler_angles[0];// Z
//     pose_in_world[1] = euler_angles[1];// Y
//     pose_in_world[2] = euler_angles[2];// X
// }

// // 将calf坐标系中的点C转换到world坐标系中
// void calfToWorld(const vector_temp<double>& base_origin, const vector_temp<double>& base_pose,
//                  const vector_temp<double>& thigh_origin, const vector_temp<double>& thigh_pose,
//                  const vector_temp<double>& calf_origin, const vector_temp<double>& calf_pose,
//                  const vector_temp<double>& point_in_calf, vector_temp<double>& point_in_world, vector_temp<double>& pose_in_world) {
//     Eigen::Matrix3d R_thigh_to_base;
//     R_thigh_to_base = Eigen::AngleAxisd(thigh_pose[2], Eigen::Vector3d::UnitX()) *
//                       Eigen::AngleAxisd(thigh_pose[1], Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(thigh_pose[0], Eigen::Vector3d::UnitZ());

//     Eigen::Matrix3d R_calf_to_thigh;
//     R_calf_to_thigh = Eigen::AngleAxisd(calf_pose[2], Eigen::Vector3d::UnitX()) *
//                       Eigen::AngleAxisd(calf_pose[1], Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(calf_pose[0], Eigen::Vector3d::UnitZ());

//     Eigen::Vector3d point_calf(point_in_calf[0], point_in_calf[1], point_in_calf[2]);
//     Eigen::Vector3d point_thigh = R_calf_to_thigh * point_calf + Eigen::Vector3d(calf_origin[0], calf_origin[1], calf_origin[2]);

//     Eigen::Vector3d point_base = R_thigh_to_base * point_thigh + Eigen::Vector3d(thigh_origin[0], thigh_origin[1], thigh_origin[2]);

//     Eigen::Matrix3d R_base_to_world;
//     R_base_to_world = Eigen::AngleAxisd(base_pose[2], Eigen::Vector3d::UnitX()) *
//                       Eigen::AngleAxisd(base_pose[1], Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(base_pose[0], Eigen::Vector3d::UnitZ());

//     Eigen::Vector3d point_world = R_base_to_world * point_base + Eigen::Vector3d(base_origin[0], base_origin[1], base_origin[2]);

//     Eigen::Matrix3d R_calf_to_world = R_base_to_world * R_thigh_to_base * R_calf_to_thigh;
//     Eigen::Vector3d euler_angles = R_calf_to_world.eulerAngles(2, 1, 0); // ZYX顺序

//     point_in_world[0] = point_world[0];
//     point_in_world[1] = point_world[1];
//     point_in_world[2] = point_world[2];

//     pose_in_world[0] = euler_angles[0];// Z
//     pose_in_world[1] = euler_angles[1];// Y
//     pose_in_world[2] = euler_angles[2];// X
// }

}  // namespace cbf
