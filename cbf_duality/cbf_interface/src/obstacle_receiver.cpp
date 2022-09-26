//
// Created by qiayuan on 2022/8/9.
//
#include "cbf_interface/obstacle_receiver.h"

#include <cbf_geometry/distance_qp_2d.h>
#include <cbf_msgs/Distances.h>

namespace cbf
{
ObstacleReceiver::ObstacleReceiver(ros::NodeHandle nh, std::shared_ptr<DualityObstacles> obstacle_ptr,
                                   SolverBase* solver)
  : obstacle_ptr_(std::move(obstacle_ptr)), solver_(solver)
{
  subscriber_ = nh.subscribe("/obstacle", 10, &ObstacleReceiver::pointsCallback, this);
  pub_ = nh.advertise<cbf_msgs::Distances>("/distances", 10);
}

void ObstacleReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                    const ReferenceManagerInterface& referenceManager)
{
  if (points_updated_)
  {
    std::lock_guard<std::mutex> lock(mutex);

    obstacle_ptr_->setPointsArray(points_array_);
    solver_->reset();
    points_updated_ = false;
  }

  size_t num_obstacles = obstacle_ptr_->getInfo().num_obstacles;
  dists_ = vector_t::Zero(num_obstacles);
  cbf_msgs::Distances distances;
  auto robot = createRobotRegion(currentState, obstacle_ptr_->getInfo().centroidal_info);
  for (size_t o = 0; o < num_obstacles; ++o)
  {
    auto obstacle = obstacle_ptr_->getRegion(o);
    Duality2d qp(robot, obstacle);
    scalar_t dist = qp.getDistance();
    if (!isnan(dist))
      dists_[o] = dist;
    if (o < msgs_size_)
      distances.data.push_back(dists_[o]);
  }
  pub_.publish(distances);
}

void ObstacleReceiver::pointsCallback(const cbf_msgs::ObstacleConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex);

  points_array_.clear();
  size_t num_points = obstacle_ptr_->getInfo().num_points;
  msgs_size_ = msg->polytopes.size();

  for (size_t o = 0; o < msg->polytopes.size(); ++o)
  {
    if (o == obstacle_ptr_->getInfo().num_obstacles)
      break;

    const auto polytopes = msg->polytopes[o].points;
    vector_t points((polytopes.size() > num_points ? num_points : polytopes.size()) * 2);
    for (size_t p = 0; p < polytopes.size(); ++p)
    {
      if (p == num_points)
      {
        ROS_WARN_STREAM("Too many point of polytopes");
        break;
      }
      points[p * 2] = polytopes[p].x;
      points[p * 2 + 1] = polytopes[p].y;
    }
    points_array_.push_back(points);
  }

  points_updated_ = true;
}

void CbfObstaclesReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                        const ReferenceManagerInterface& referenceManager)
{
  ObstacleReceiver::preSolverRun(initTime, finalTime, currentState, referenceManager);

  scalar_t threshold = 0.06;
  size_t num_obstacles = obstacle_ptr_->getInfo().num_obstacles;
  for (size_t o = 0; o < num_obstacles; ++o)
    if (dists_[o] > threshold)
      dists_[o] -= threshold;
    else
      dists_[o] = 0;
  static_cast<CbfObstacles&>(*obstacle_ptr_).setDists(initTime, dists_);
}

}  // namespace cbf
