
#pragma once

#include "PathPlanningAlgos/generateTraj.h"
#include <ros/ros.h>



#include "potential_fields/gauss/gaussian_potential_field.hpp"
namespace ImmersionBot {

  class serviceHandler {
  
    private:
      ros::NodeHandle _nh;
      
      //these handle all the computations
      ros::ServiceServer _generate_traj_srv;
      ros::ServiceServer _modify_traj_srv;

      //This publishes the trajectory to the mpcc
      ros::Publisher _trajectory_pub;

      PotentialFields::GPF _ptl_fld; 

      std::vector<Eigen::Vector2d> _trajectory
      
      bool modifyTrajectory(ImmersionBot::modifyTraj::Request &req, ImmersionBot::modifyTraj::res &res);

      bool generateTrajectory(PathPlanningAlgos::generateTraj::Request &req, PathPlanningAlgos::generateTraj::Response &res);

      vector<double> compute_arclength(vector<Eigen::Vector2d> trajectory);
      
    public:
      serviceHandler(ros::NodeHandle& nh);
      ~serviceHandler()=default;

  };

}
