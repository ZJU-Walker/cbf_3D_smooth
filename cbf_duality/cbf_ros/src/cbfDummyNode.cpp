/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
// TODO: This related to LeggedController.cpp (legged_controllers/src)
#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
// TODO: change file
// #include <ocs2_legged_robot/LeggedRobotInterface.h>
#include "cbf_controllers/CbfControllers.h"

#include "cbf_interface/DCbfLeggedInterface.h"
#include "cbf_interface/ObstacleReceiver.h"


#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "cbf_ros/visualization/cbfVisualization.h"

using namespace ocs2;
using namespace cbf;

int main(int argc, char** argv) {
  std::cout << "[cbfDummyNode.cpp] start dummy node" << std::endl;
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);

  // Robot interface
  // TODO may need to change interface
  // LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
  // DualityLeggedInterface interface(taskFile, urdfFile, referenceFile, false);
  // interface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, false);

    DCbfDualityLeggedInterface interface(taskFile, urdfFile, referenceFile, false);
    interface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, false);

  std::cout << "[cbfDummyNode.cpp] dummy node step 1" << std::endl;
  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  std::cout << "[cbfDummyNode.cpp] dummy node step 2" << std::endl;

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                       interface.modelSettings().contactNames3DoF);
  auto cbfVisualization_ = std::make_shared<cbfVisualization>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle);
  // TODO: selfCollisionVisualiztion can be added? 

  std::cout << "[cbfDummyNode.cpp] dummy node step 3" << std::endl;

  // Dummy legged robot
  MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                                               interface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummySimulator.subscribeObservers({cbfVisualization_});


  std::cout << "[cbfDummyNode.cpp] dummy node step 4" << std::endl;
  // Initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  initObservation.mode = ModeNumber::STANCE;
  std::cout << "[cbfDummyNode.cpp] dummy node step 5" << std::endl;
  // Initial command
  TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input});
  std::cout << "[cbfDummyNode.cpp] dummy node step 6" << std::endl;
  // run dummy
  leggedRobotDummySimulator.run(initObservation, initTargetTrajectories);
  std::cout << "[cbfDummyNode.cpp] dummy node step 7" << std::endl;
  // Successful exit
  return 0;
}
