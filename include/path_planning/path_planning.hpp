#pragma once

#include "potential_fields/gauss/gaussian_potential_field.hpp"
#include "path_planning.hpp"
#include <ros/ros.h>
#include "tracking/obstacle_manager.hpp"
#include "objects/coord.hpp"
#include <memory>

class PathPlanningNode
{
public:
    PathPlanningNode(ros::NodeHandle& nh);
    ~PathPlanningNode();

private:
    // ROS NodeHandle
    ros::NodeHandle _nh;
    
    // Trackers
    std::unique_ptr<WorldState::ObstacleTracker> _obstacle_tracker;
    std::unique_ptr<WorldState::RobotTracker> _robot_tracker;
    
    // Parameters for potential field
    double _resolution;
    double _attractive_gain;
    double _obstacle_amplitude;
    double _obstacle_sigma;
    double _robot_scan_hz;
    
    // Topic names
    std::string _robot_topic;
    std::string _gen_traj_topic;
    std::string _modify_traj_topic;
    
    // Service servers
    ros::ServiceServer _gen_traj_srv;
    ros::ServiceServer _modify_traj_srv;
    
    // Service callbacks
    bool generateTrajSrv(uvatraj_msgs::RequestTraj::Request &req, 
                        uvatraj_msgs::RequestTraj::Response &res);
    
    bool modifyTrajSrv(uvatraj_msgs::ModifyTraj::Request &req,
                      uvatraj_msgs::ModifyTraj::Response &res);
};