
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
#include "ImmersionBot/generateTraj.h"

ImmersionBot::serviceHandler::serviceHandler(ros::NodeHandle& nh) : _nh("~"){

  _generate_traj_srv = nh.advertiseService("/generateTrajectory", &ImmersionBot::serviceHandler::generateTrajectory, this);
  _modify_traj_srv = nh.advertiseService("/modifyTrajectory", &ImmersionBot::serviceHandler::modifyTrajectory, this);

  _trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectoryConstPtr>("/TransferTrajectory", 10);

}

bool ImmersionBot::serviceHandler::generateTrajectory(PathPlanningAlgos::generateTraj::Request &req,
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

   _trajectory = _ptl_fld.generateTrajectory(pos_r,learnedPreferences, goal);
  
   //TODO: use clampedBSpline

   _trajectory = utils::calculateClampedBSpline(_trajectory);

   res.trajectory.resize(trajectory.size());
    for (auto& pt : _trajectory){
    size_t idx = &pt - &trajectory[0];
    res.trajectory[idx].x = pt[0];
    res.trajectory[idx].y = pt[1];
    res.trajectory[idx].z = 0;
    }

    //find arclength
    
    //publish path for MPCC
    _trajectory_pub.publish(res.trajectory);

    //publilsh the arclength

    return true;
}


bool ImmersionBot::modifyTrajectory(ImmersionBot::modifyTraj::Request &req, ImmersionBot::modifyTraj::res &res){


  //get the index of the point that was altered
  Eigen::Vector2d modified_pt(req.modified_pt.x,req.modified_pt.y);
  int index = req.index;
  if(!running){

    //recalculate the arclengthv
    _trajectory[index][0] = modified_pt[0];
    _trajectory[index][1] = modified_pt[1];

    //recompute arclength
    vector<double> arclength = compute_arclength() 

  }
  //If not running:
    // Assume point is at index i 
    // recompute the arclength from point i-1 to i and i to i+1
    // smooth from i -> end
    //


}



int main(int argc, char** argv){

  ros::init(argc,argv,"PathPlanner");
  ros::NodeHandle nh;
  ImmersionBot::serviceHandler serviceHandler(nh);

  ros::spin();

}
