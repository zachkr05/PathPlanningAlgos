#pragma once

#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "types/obstacle.hpp"
#include "types/preferences.hpp"
#include <map>
#include <vector>
#include "types/goal.hpp"



namespace PotentialFields {



  using Obstacle = Environment::Obstacle;
  using Map = std::map<UserPreferences::PotentialFieldParams, std::vector<Obstacle>>;
  using Preferences = UserPreferences::PotentialFieldParams;
  using Goal = World::Goal;
  using Point = Eigen::Vector2d;



  class GPF {
    public:

      GPF() = default;
      
      const int MAX_PTS=100;
      const double RESOLUTION=0.03;

      const std::vector<Eigen::Vector2d> generateTrajectory(const Eigen::Vector2d& pos_r, const Map& obstacles, const Goal& goal);

      const Eigen::Vector2d calculateAttractiveGradient(const Eigen::Vector2d& point, const Goal& goal);

      const Eigen::Vector2d calculateTotalGradient(const Eigen::Vector2d& point, const Map& obstacles, const Goal& goal);

      const Eigen::Vector2d calculateSummedRepulsiveGradient(const Eigen::Vector2d& point, const Map& obstacles, const Goal& goal);
      const double calculateSignedDistance(const Eigen::Vector2d& p,const Obstacle& obstacle) const noexcept;
      Point calculateRepulsiveGradient(const Eigen::Vector2d& point, const Preferences& params, const std::vector<Obstacle>& obstacle, const Goal& goal);

      const Eigen::Vector2d calculateNormalVector(const Eigen::Vector2d& p, const Obstacle& obstacle) const noexcept;
  };

}
