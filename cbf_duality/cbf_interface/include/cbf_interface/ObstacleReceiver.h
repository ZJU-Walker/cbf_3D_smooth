//
// Created by Ke Wang on 2023/7/20.
//

#pragma once

#include <mutex>

#include <cbf_msgs/Obstacle.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "cbf_interface/Obstacle.h"

namespace cbf {
using namespace ocs2;

class ObstacleReceiver : public SolverSynchronizedModule {
 public:
  ObstacleReceiver(ros::NodeHandle nh, std::shared_ptr<DualityObstacles> obstacle, SolverBase* solver);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 protected:
  std::shared_ptr<DualityObstacles> obstaclePtr_;
  vector_array_t dists_;

 private:
  void obstacleparamCallback(const cbf_msgs::ObstacleConstPtr& msg);

  SolverBase* solver_;

  ros::Subscriber subscriber_;
  ros::Publisher pub_;

  std::mutex mutex_;
  std::atomic_bool obstacleUpdated_{};
  vector_array2_t obstacle3d_;//contain param for obstacles, each element param contains pose and size
  size_t msgsSize_{};
};

class CbfObstaclesReceiver : public ObstacleReceiver {
 public:
  using ObstacleReceiver::ObstacleReceiver;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;
};

}  // namespace cbf
