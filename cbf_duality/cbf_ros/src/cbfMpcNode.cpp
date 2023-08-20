/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ros/init.h>
#include <ros/package.h>

// #include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "cbf_controllers/CbfControllers.h"

#include "cbf_interface/DCbfLeggedInterface.h"
#include "cbf_interface/ObstacleReceiver.h"

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>
#include <ocs2_sqp/SqpMpc.h>

#include "ocs2_legged_robot_ros/gait/GaitReceiver.h"
#include <iostream>

using namespace ocs2;
using namespace cbf;

int main(int argc, char** argv) {
  std::cout << "[cbfMpcNode.cpp] main step 1" << std::endl;
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_mpc");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  bool multiplot = false;
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/multiplot", multiplot);
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);

  std::cout << "[cbfMpcNode.cpp] main step 2" << std::endl;

  // Robot interface
  // LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
  // DualityLeggedInterface interface(taskFile, urdfFile, referenceFile, false);
  // interface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, false);

  DCbfDualityLeggedInterface interface(taskFile, urdfFile, referenceFile, false);
  interface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, false);

  // DualityLeggedInterface interface(taskFile, urdfFile, referenceFile, false);
  // interface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, false);

  std::cout << "[cbfMpcNode.cpp] main step 3" << std::endl;

  // // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  std::cout << "[cbfMpcNode.cpp] main step 4" << std::endl;

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  std::cout << "[cbfMpcNode.cpp] main step 5" << std::endl;

  // MPC
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  std::cout << "[cbfMpcNode.cpp] main step 6" << std::endl;

  // obstalce receiver

  // auto obstacle_receiver =
  //   std::make_shared<ObstacleReceiver>(nodeHandle, dynamic_cast<DualityLeggedInterface&>(interface).getObstacles(), mpc.getSolverPtr());
  // mpc.getSolverPtr()->addSynchronizedModule(obstacle_receiver);

  auto obstacle_receiver =
    std::make_shared<CbfObstaclesReceiver>(nodeHandle, dynamic_cast<DCbfDualityLeggedInterface&>(interface).getObstacles(), mpc.getSolverPtr());
  mpc.getSolverPtr()->addSynchronizedModule(obstacle_receiver);
  std::cout << "[cbfMpcNode.cpp] main step 7" << std::endl;
  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);
  std::cout << "[cbfMpcNode.cpp] main step 8" << std::endl;
  // Successful exit
  return 0;
}
