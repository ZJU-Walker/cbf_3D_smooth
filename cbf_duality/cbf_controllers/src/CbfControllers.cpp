//
// Created by qiayuan on 2022/7/16.
//

#include "cbf_controllers/CbfControllers.h"

#include "cbf_interface/DCbfLeggedInterface.h"
#include "cbf_interface/ObstacleReceiver.h"

#include <pluginlib/class_list_macros.hpp>

namespace cbf {

void DualityController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                             bool verbose) {
  leggedInterface_ = std::make_shared<DualityLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void DualityController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto obstacle_receiver =
      std::make_shared<ObstacleReceiver>(nh, dynamic_cast<DualityLeggedInterface&>(*leggedInterface_).getObstacles(), mpc_->getSolverPtr());
  mpc_->getSolverPtr()->addSynchronizedModule(obstacle_receiver);
}



void DCbfDualityController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                 bool verbose) {
  leggedInterface_ = std::make_shared<DCbfDualityLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void DCbfDualityController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto obstacleReceiver = std::make_shared<CbfObstaclesReceiver>(
      nh, dynamic_cast<DCbfDualityLeggedInterface&>(*leggedInterface_).getObstacles(), mpc_->getSolverPtr());
  mpc_->getSolverPtr()->addSynchronizedModule(obstacleReceiver);
}

}  // namespace cbf

PLUGINLIB_EXPORT_CLASS(cbf::DualityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cbf::DCbfDualityController, controller_interface::ControllerBase)
