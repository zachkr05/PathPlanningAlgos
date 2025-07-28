#include "path_planning_node.hpp"

PathPlanningNode::PathPlanningNode(ros::NodeHandle& nh) : _nh("~")
{
    // Load all parameters from ROS parameter server
    _nh.param("robot_scan_hz", _robot_scan_hz, 1.0);
    _nh.param("trajectory_resolution", _resolution, 0.05);
    _nh.param("attractive_gain", _attractive_gain, 1.0);
    _nh.param("obstacle_amplitude", _obstacle_amplitude, 1.0);
    _nh.param("obstacle_sigma", _obstacle_sigma, 0.5);
    
    // Topic parameters
    _nh.param<std::string>("robot_topic", _robot_topic, "/vicon/Rosbot_AR_2/Rosbot_AR_2");
    _nh.param<std::string>("gen_traj_topic", _gen_traj_topic, "/gen_traj");
    _nh.param<std::string>("modify_traj_topic", _modify_traj_topic, "/modify_traj");
    
    // Log the loaded parameters
    ROS_INFO("=====================================");
    ROS_INFO("Path Planning Node Parameters:");
    ROS_INFO("  - Robot scan Hz: %.2f", _robot_scan_hz);
    ROS_INFO("  - Trajectory resolution: %.3f", _resolution);
    ROS_INFO("  - Attractive gain: %.2f", _attractive_gain);
    ROS_INFO("  - Obstacle amplitude: %.2f", _obstacle_amplitude);
    ROS_INFO("  - Obstacle sigma: %.2f", _obstacle_sigma);
    ROS_INFO("  - Robot topic: %s", _robot_topic.c_str());
    ROS_INFO("=====================================");
    
    // Initialize obstacle tracking
    _obstacle_tracker = std::make_unique<WorldState::ObstacleTracker>();
    _obstacle_tracker->initialize(nh, _robot_scan_hz);
    ROS_INFO("Obstacle tracker initialized");
    
    // Initialize robot tracking
    _robot_tracker = std::make_unique<WorldState::RobotTracker>(nh, _robot_topic);
    ROS_INFO("Robot tracker initialized");
    
    // Advertise services
    _gen_traj_srv = nh.advertiseService(_gen_traj_topic, &PathPlanningNode::generateTrajSrv, this);
    _modify_traj_srv = nh.advertiseService(_modify_traj_topic, &PathPlanningNode::modifyTrajSrv, this);
    
    ROS_INFO("Path Planning Node ready!");
}

PathPlanningNode::~PathPlanningNode()
{
    // Cleanup handled automatically by unique_ptr
}

bool PathPlanningNode::generateTrajSrv(uvatraj_msgs::RequestTraj::Request &req, 
                                      uvatraj_msgs::RequestTraj::Response &res)
{
    // Using member variables
    const auto& obstacles = _obstacle_tracker->getObstacles();
    
    // Extract obstacle positions for path planning
    std::vector<World::Obstacle> world_obstacles;
    for (const auto& [id, obstacle] : obstacles) 
    {
        World::Obstacle world_obs;
        world_obs.id = id;
        world_obs.setPosition(obstacle.getPosition());
        world_obs.setParams(_obstacle_amplitude, _obstacle_sigma);
        world_obstacles.push_back(world_obs);
    }
    
    // Generate the trajectory
    World::Goal goal;
    Eigen::Vector2d goal_pos(-req.goal.z, req.goal.y);
    goal.setPosition(goal_pos);
    goal.setAttractiveGain(_attractive_gain);
    
    PotentialField::GaussianPotentialField GPF(world_obstacles, goal);
    
    // Get robot position
    Eigen::Vector2d robot_pos = _robot_tracker->position();
    
    // Generate trajectory
    Eigen::RowVectorXd xs, ys, ss;
    GPF.generateTrajectory(robot_pos, _resolution, xs, ys, ss);
    
    ROS_INFO("Generated trajectory with %ld points", xs.size());
    return true;
}


bool MPCCROS::modifyTrajSrv(uvatraj_msgs::ExecuteTraj::Request &req, uvatraj_msgs::ExecuteTraj::Response &res)
{

	if (_is_executing){

		printf("Executing trajectory blend.");	
		_old_ref = _ref;
		_old_ref_len = _ref_len;
		_blend_traj_curr_s = get_s_from_state(_ref, _true_ref_len);


		 printf("Current s from state: %.3f", _blend_traj_curr_s);
		_transition_start_time = ros::Time::now().toSec();
		_transition_duration = 1.0;
	 	_in_transition = true;

		_blend_new_s = std::ceil(_blend_traj_curr_s/resolution);

		printf("Blend target s: %.3f", _blend_new_s);	
	}

	_prev_ref     = _ref;
    _prev_ref_len = _true_ref_len;	

	int N = req.ctrl_pts.size();

	Eigen::RowVectorXd* ss = new Eigen::RowVectorXd(N);
    Eigen::RowVectorXd* xs = new Eigen::RowVectorXd(N);
    Eigen::RowVectorXd* ys = new Eigen::RowVectorXd(N);	

	(*xs)(0) = req.ctrl_pts[0].x;
    	(*ys)(0) = req.ctrl_pts[0].y;
	(*ss)(0) = 0.0;
     	for (int i = 1; i < N; ++i) {
    		(*xs)(i) = req.ctrl_pts[i].x;
    		(*ys)(i) = req.ctrl_pts[i].y;
		
    		double dx = (*xs)(i) - (*xs)(i-1);
    		double dy = (*ys)(i) - (*ys)(i-1);
    		(*ss)(i) =(*ss)(i-1) + std::hypot(dx, dy);
	}


	//_ref_len = resampleByArcLength(splineX,splineY, (ss)(0), (ss)(ss.size()-1));

    	//_ref_len = (*ss)(N - 1);
    	//_true_ref_len = _ref_len;
	


	std::vector<unsigned int> duplicate_pts;

	bool valid = true;
	for (int i = 1; i < ss->size(); ++i) {
    		if ((*ss)(i) <= (*ss)(i-1)) {
       			ROS_WARN_STREAM("Non-increasing arc length at i=" << i << ": ss(i)=" << (*ss)(i) << ", ss(i-1)=" << (*ss)(i-1));
    			duplicate_pts.push_back(i);
		} 
	}
	

	
	removePts(xs, duplicate_pts);
	removePts(ys, duplicate_pts);
	removePts(ss, duplicate_pts);


	//int smoothing_window = 25;
	//smoothPathMovingAverage((*xs),(*ys),(*xs),(*ys), smoothing_window);

	recalculateArcLength((*xs),(*ys),(*ss));

}

bool PathPlanningNode::modifyTrajSrv(uvatraj_msgs::ModifyTraj::Request &req,
                                    uvatraj_msgs::ModifyTraj::Response &res)
{
    // Access obstacles the same way
    const auto& obstacles = _obstacle_tracker->getObstacles();
    
    // TODO: implement modify trajectory functionality
    // - Clamped B-spline smoothing
    // - Send to MPCC
    
    return true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    
    try 
    {
        PathPlanningNode path_planner(nh);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Path Planning Node failed: %s", e.what());
        return 1;
    }
    
    return 0;
}