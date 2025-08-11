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


  class GPF {
    public:
      GPF() = default;
      using Obstacle = Environment::Obstacle;
      using Map = std::map<UserPreferences::PotentialFieldParams, std::vector<Obstacle>>;
      using Preferences = UserPreferences::PotentialFieldParams;
      using Goal = World::Goal;
      using Point = Eigen::Vector2d;
      //ros::NodeHandle nh("~");

      const int MAX_PTS=100;
      const double RESOLUTION=100;
  //    nh.getParam("max_pts", MAX_PTS);
    //  nh.getParam("resolution", RESOLUTION) 
    
      
      const std::vector<Eigen::Vector2d> generateTrajectory(Eigen::Vector2d& pos_r, Map& obstacles, Goal& goal);


      const Eigen::Vector2d calculateAttractiveGradient(Eigen::Vector2d& point, Goal& goal);
    
      const Eigen::Vector2d calculateTotalGradient(Eigen::Vector2d& point, Map& obstacles,  Goal& goal);

      const Eigen::Vector2d calculateSummedRepulsiveGradient(Eigen::Vector2d& point, Map& obstacles);
  const double calculateSignedDistance(const Eigen::Vector2d& p,const Obstacle& obstacle) const noexcept;
  Point calculateRepulsiveGradient(Eigen::Vector2d& point, const Preferences& params, const std::vector<Obstacle>& obstacle);

  const Eigen::Vector2d calculateNormalVector(const Eigen::Vector2d& p, const Obstacle& obstacle) const noexcept;
  };

}
