//
// Created by Ke Wang on 2023/7/21.
//
#include "cbf_interface/ObstacleReceiver.h"

#include <cbf_geometry/DistanceQp3d.h>
#include <cbf_msgs/Distances.h>

#include <utility>

namespace cbf {

// 3d
ObstacleReceiver::ObstacleReceiver(ros::NodeHandle nh, std::shared_ptr<DualityObstacles> obstacle_ptr, SolverBase* solver)
    : obstaclePtr_(std::move(obstacle_ptr)), solver_(solver) {
  // std::cout << "========================================================" << std::endl;
  // std::cout << "ObstacleReceiver.cpp ObstacleReceiver begins" << std::endl;
  // std::cout << "========================================================" << std::endl;
  subscriber_ = nh.subscribe("/obstacle", 10, &ObstacleReceiver::obstacleparamCallback, this);
  pub_ = nh.advertise<cbf_msgs::Distances>("/distances", 10);
  // std::cout << "========================================================" << std::endl;
  // std::cout << "ObstacleReceiver.cpp ObstacleReceiver ends" << std::endl;
  // std::cout << "========================================================" << std::endl;
}

// 3d
void ObstacleReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& currentState,
                                    const ReferenceManagerInterface& /*referenceManager*/) {
  // std::lock_guard<std::mutex> lock(mutex_);

  if (obstacleUpdated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacleUpdated_ = false;
    obstaclePtr_->setObstacle3d(obstacle3d_);
    solver_->reset();
  }

  size_t num_obstacles = obstaclePtr_->getInfo().numObstacles;
  size_t num_robots = obstaclePtr_->getInfo().numRobots;

  dists_ = vector_array_t(num_obstacles, vector_t::Zero(num_robots));
  cbf_msgs::Distances distances;
  auto robot = createRobotRegion3d(currentState, obstaclePtr_->getInfo().centroidalInfo);
  for (size_t o = 0; o < num_obstacles; ++o) {
    for (size_t r = 0; r < num_robots; ++r){
      auto obstacle = obstaclePtr_->getRegion3d(o);
      Duality3d qp(robot[r], obstacle);
      scalar_t dist = qp.getDistance();
      if(!isnan(dist)) dists_[o][r] = dist;
    }
    if (o < msgsSize_) {
      if (num_robots == 1) {
        // std::cout << "***************/********"  << std::endl;
        // std::cout << "*****Only body involved" << std::endl;
        // std::cout << "dist_body: " << dists_[o][0] << std::endl;
        // std::cout << "base_pose: " << centroidal_model::getBasePose(currentState, obstaclePtr_->getInfo().centroidalInfo) << std::endl;
        // std::cout << "Robot body A:\n " << robot[0].getA() << std::endl;
        // std::cout << "Robot body b:\n " << robot[0].getB() << std::endl;
        // std::cout << "Robot body pose:\n " << centroidal_model::getBasePose(currentState, obstaclePtr_->getInfo().centroidalInfo) << std::endl;
        // std::cout << "***********************"  << std::endl;
        distances.dist_body.push_back(dists_[o][0]);
        distances.dist_thigh1.push_back(dists_[o][0]);
        distances.dist_thigh2.push_back(dists_[o][0]);
        distances.dist_thigh3.push_back(dists_[o][0]);
        distances.dist_thigh4.push_back(dists_[o][0]);
        distances.dist_calf1.push_back(dists_[o][0]);
        distances.dist_calf2.push_back(dists_[o][0]);
        distances.dist_calf3.push_back(dists_[o][0]);
        distances.dist_calf4.push_back(dists_[o][0]);
      }
      else if (num_robots == 9) {
        // std::cout << "***********************"  << std::endl;
        // std::cout << "dist_body: " << dists_[o][0] << std::endl;
        // std::cout << "dist_calf_1: " << dists_[o][1] << std::endl;
        // std::cout << "base_pose_x_y_z: " << centroidal_model::getBasePose(currentState, obstaclePtr_->getInfo().centroidalInfo)[0] << "," << centroidal_model::getBasePose(currentState, obstaclePtr_->getInfo().centroidalInfo)[1] << "," << centroidal_model::getBasePose(currentState, obstaclePtr_->getInfo().centroidalInfo)[2] << std::endl;
        // std::cout << "***********************"  << std::endl;
        distances.dist_body.push_back(dists_[o][0]);
        distances.dist_thigh1.push_back(dists_[o][1]);
        distances.dist_thigh2.push_back(dists_[o][2]);
        distances.dist_thigh3.push_back(dists_[o][3]);
        distances.dist_thigh4.push_back(dists_[o][4]);
        distances.dist_calf1.push_back(dists_[o][5]);
        distances.dist_calf2.push_back(dists_[o][6]);
        distances.dist_calf3.push_back(dists_[o][7]);
        distances.dist_calf4.push_back(dists_[o][8]);
      }
      else if (num_robots == 5) {
        distances.dist_body.push_back(dists_[o][0]);
        distances.dist_thigh1.push_back(dists_[o][1]);
        distances.dist_thigh2.push_back(dists_[o][2]);
        distances.dist_thigh3.push_back(dists_[o][3]);
        distances.dist_thigh4.push_back(dists_[o][4]);
        distances.dist_calf1.push_back(dists_[o][0]);
        distances.dist_calf2.push_back(dists_[o][0]);
        distances.dist_calf3.push_back(dists_[o][0]);
        distances.dist_calf4.push_back(dists_[o][0]);
      }
      else if (num_robots == 8) {
        distances.dist_body.push_back(dists_[o][0]);
        distances.dist_thigh1.push_back(dists_[o][0]);
        distances.dist_thigh2.push_back(dists_[o][0]);
        distances.dist_thigh3.push_back(dists_[o][0]);
        distances.dist_thigh4.push_back(dists_[o][0]);
        distances.dist_calf1.push_back(dists_[o][0]);
        distances.dist_calf2.push_back(dists_[o][0]);
        distances.dist_calf3.push_back(dists_[o][0]);
        distances.dist_calf4.push_back(dists_[o][0]);
      }
      else {
        std::cout << "num_robots invalid" << std::endl;
      }
    }
  }
  pub_.publish(distances);
}

void ObstacleReceiver::obstacleparamCallback(const cbf_msgs::ObstacleConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  obstacle3d_.resize(msg->polytopes.size());
  obstacle3d_.clear();
  size_t num_points = 6;// only consider cuboid, originally obstaclePtr_->getInfo().numPoints (in 2d)
  msgsSize_ = msg->polytopes.size();

  for (size_t o = 0; o < msg->polytopes.size(); ++o) {
    if (o == obstaclePtr_->getInfo().numObstacles) break;

    const auto polytopes3d_position = msg->polytopes[o].pose.position;
    const auto polytopes3d_orientation = msg->polytopes[o].pose.orientation;
    const auto polytopes3d_size = msg->polytopes[o].size;
    
    vector_t pose(6);
    vector_t size(3);
    pose << polytopes3d_position.x, polytopes3d_position.y, polytopes3d_position.z, polytopes3d_orientation.z, polytopes3d_orientation.y, polytopes3d_orientation.x;// TODO: change to ZYX here
    size << polytopes3d_size.x, polytopes3d_size.y, polytopes3d_size.z; 

    obstacle3d_[o].push_back(pose);
    obstacle3d_[o].push_back(size);
  }

  obstacleUpdated_ = true;
}

void CbfObstaclesReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                        const ReferenceManagerInterface& referenceManager) {
  ObstacleReceiver::preSolverRun(initTime, finalTime, currentState, referenceManager);

  scalar_t threshold = 0.05;
  size_t num_obstacles = obstaclePtr_->getInfo().numObstacles;
  size_t num_robots = obstaclePtr_->getInfo().numRobots;
  for (size_t o = 0; o < num_obstacles; ++o) {
    for (size_t r = 0; r < num_robots; ++r) {
      if (dists_[o][r] > threshold)
        dists_[o][r] -= threshold;
      else
        dists_[o][r] = 0;
    }
  }
  std::cout << "[ObsatcleReceiver] dist_: " << dists_[0][0] << std::endl;
  dynamic_cast<CbfObstacles&>(*obstaclePtr_).setDists(initTime, dists_);
}

}  // namespace cbf
