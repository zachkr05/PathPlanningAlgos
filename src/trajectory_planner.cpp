
#include "potential_fields/gaussian_potential_field.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>


namespace PathPlanning{

  std::vector<PathPlanning::Obstacles> _obstacles;
  PathPlanning::PotentialFields::GaussianPotentialField _ptl_field;
  

  void scanForViconObstacles()
  {
      std::vector<ros::master::TopicInfo> topics;
      if (ros::master::getTopics(topics)) {
          std::regex pattern("^/vicon/obstacle_\\d+/obstacle_\\d+$");

          for (const auto& t : topics) {
              if (std::regex_match(t.name, pattern) && known_topics.find(t.name) == known_topics.end()) {
                  // Extract obstacle name
                  size_t last_slash = t.name.rfind('/');
                  std::string obs_name = t.name.substr(last_slash + 1);

                  Obstacle obs{obs_name, t.name};
                  obstacle_list.push_back(obs);
                  known_topics.insert(t.name);

                  ROS_INFO_STREAM("Detected new Vicon obstacle: " << obs.name << " on topic " << obs.topic);
              }
          }
      }
  }

  bool handlePlanTrajectory(path_planning::PlanTrajectory::Request  &req,
				  path_planning::PlanTrajectory::Response &res){

    Eigen::Vector2d start(req.robot_pos.x, req.robot_pos.y);
    Eigen::Vector2d goal(req.goal_pos.x, req.goal_pos.y);
    PathPlanning::Goal goal {Eigen::Vector2d(-req.goal.z,req.goal.y), 0.5};
	  PathPlanning::GaussianPotentialField GPR(_obstacles, goal); 

    _ptl_field.setGoal(goal);
    _ptl_field.setObstacles(_obstacles);

    Eigen::RowVector xs;
    Eigen::RowVector ys;
    Eigen::RowVector ss;

    PathPlanning::PotentialFields::GaussianPotentialField generateTrajectory(start, 0.5, _ptl_field,xs,ys,ss);

    for (int i = 0; i<xs.size(); i++){
      
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_trajectory_server");
  ros::NodeHandle nh;

  ros::ServiceServer server = nh.advertiseService("/plan_trajectory", PathPlanning::handlePlanTrajectory);
  ROS_INFO("Trajectory planning service 'plan_trajectory' is ready."); 

  ros::Rate rate(1.0);  
  while (ros::ok()) {
    PathPlanning::scanForViconObstacles();
    ros::spinOnce();
    rate.sleep();
  }

  ros::spin();
  return 0;
}
