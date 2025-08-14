
#pragma once

#include "PathPlanningAlgos/generateTraj.h"
#include <ros/ros.h>



#include "potential_fields/gauss/gaussian_potential_field.hpp"
namespace PathPlanning {

  class serviceHandler {
  
    private:
      ros::NodeHandle _nh;
      ros::ServiceServer _generate_traj_srv;
      PotentialFields::GPF _ptl_fld; 

      bool generateTrajectory(PathPlanningAlgos::generateTraj::Request &req, PathPlanningAlgos::generateTraj::Response &res);

      
    public:
      serviceHandler(ros::NodeHandle& nh);
      ~serviceHandler()=default;

  };

}
