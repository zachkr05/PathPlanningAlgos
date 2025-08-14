
#include "geometry_msgs/TransformStamped.h"
#include <string>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "potential_fields/gauss/gaussian_potential_field.hpp"
#include <map>
#include <vector>
#include "types/goal.hpp"
#include <thread>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <future>
#include "path_planning/path_planning.hpp"

#include <string>
#include <vector>
#include <map>
#include "PathPlanningAlgos/generateTraj.h"

PathPlanning::serviceHandler::serviceHandler(ros::NodeHandle& nh) : _nh("~"){

  _generate_traj_srv = _nh.advertiseService("/generateTrajectory", &PathPlanning::serviceHandler::generateTrajectory, this);

}

bool PathPlanning::serviceHandler::generateTrajectory(PathPlanningAlgos::generateTraj::Request &req,
                        PathPlanningAlgos::generateTraj::Response &res){

  Eigen::Vector2d pos_g (req.goal.transform.translation.x, req.goal.transform.translation.y);
  double k_att = req.goal.k_att;

  Eigen::Vector2d pos_r (req.robot.transform.translation.x, req.robot.transform.translation.y);
  
  World::Goal goal{pos_g,k_att};

  std::vector<UserPreferences::PotentialFieldParams> key;
  for (auto& param : req.params){

    double amplitude = param.amplitude;
    double sigma = param.sigma;
    std::string id = param.id;
    UserPreferences::PotentialFieldParams obsParameter{amplitude,sigma,id};

    key.push_back(obsParameter);
  }

  std::vector<std::vector<Environment::Obstacle>> values;
  for (auto& value : req.values){

    std::vector<Environment::Obstacle> parsedObstacles;
    for (auto& obs : value.obstacles) {
      std::string id = obs.id;
      Eigen::Vector2d center(obs.transform.translation.x, obs.transform.translation.y);
      double radius = obs.radius;

      Environment::Obstacle obstacle{id,center,radius};

      parsedObstacles.push_back(obstacle);
    }

    values.push_back(parsedObstacles);

  }

  std::map<UserPreferences::PotentialFieldParams, std::vector<Environment::Obstacle>> learnedPreferences;


   for (size_t i = 0; i < key.size(); ++i) {
      learnedPreferences[key[i]] = values[i];
  }




   std::vector<Eigen::Vector2d> trajectory = _ptl_fld.generateTrajectory(pos_r,learnedPreferences, goal);


   res.trajectory1.resize(trajectory.size());
  for (auto& pt : trajectory){
    size_t idx = &pt - &trajectory[0];
    res.trajectory1[idx].x = pt[0];
    res.trajectory1[idx].y = pt[1];
    res.trajectory1[idx].z = 0;
  }

  return true;
}

int main(int argc, char** argv){

  ros::init(argc,argv,"PathPlanner");
  ros::NodeHandle nh;
  PathPlanning::serviceHandler serviceHandler(nh);

  ros::spin();

}
