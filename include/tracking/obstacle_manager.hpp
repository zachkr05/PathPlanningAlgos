#pragma once

#include <map>
#include <string>
#include <cstddef>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include "potential_fields/gauss/goal_obstacle.hpp"  // defines Obstacle

namespace Environment {

  class ObstacleManager {
  public:

    /// Map type used for obstacle storage.
    using Map = std::map<std::string, Obstacle>;

    Map obstacles; // stores all obstacles

    obstacleCallback(std::string_view id, rps::geometry_msgs::Transformstamped::ConstPtr& msg){
      if (obstacles.find(id) == obstacles.end()){
        obstacles[id] = Obstacle();
      }
      obstacles[id].setPosition(Utils::PathPlanning::to2D(*msg));
    }

  private:

  } //class ObstacleManager 
} // namespace Environment
