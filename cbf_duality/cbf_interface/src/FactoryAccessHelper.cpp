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

}  // namespace cbf
