#pragma once

//TODO:
//import the structs
#include <vector>

namespace PotentialField{

class GaussianPotentialField
{

private:
    std::vector<Obstacle> _obstacles;
    Goal                  _goal;

public:
	explicit GaussianPotentialField (std::vector<Obstacle> obs, Goal g = {}):
		_obstacles(std::move(obs)), _goal(std::move(g)) {}


	/*
	 *
	 *
	 *	Mutators
	 *
	 *
	*/

	inline void setGoal(const Goal& g) noexcept {_goal = g;}	
	inline void setObstacles(const std::vector<Obstacle>& obs) noexcept {_obstacles = obs;}
	
	inline Eigen::Vector2d getGoal() noexcept {return _goal.getPosition();}

	inline void addObstacle(const Obstacle& o) {_obstacles.push_back(o);}
	inline void clearObstacles() noexcept {_obstacles.clear();}

	inline double getTotalPotential(Eigen::Vector2d& p) const noexcept{
		double ptl = _goal.getAttractiveForce(p);
		for (const auto& obs : _obstacles) ptl += obs.getPotential(p);
		return ptl;
	}

	inline Eigen::Vector2d getTotalGradient(Eigen::Vector2d point){
		Eigen::Vector2d g = _goal.getAttractiveGradient(point);          // (−∇U_att)
		for (const auto& obs : _obstacles) g += obs.getGradient(point, _goal.getPosition());
		return g;
	}
    
  inline void generateTrajectory(const Eigen::Vector2d& start, double resolution, PathPlanning::GaussianPotentialField GPR, 	Eigen::RowVectorXd& xs, Eigen::RowVectorXd& ys, Eigen::RowVectorXd& ss){

    Eigen::Vector2d current_pt = start;

    int N = 1000;
    int ctr = 0;
    
    ys.resize(N);
    xs.resize(N);
    ss.resize(N);	

    xs(0) = start(0);
    ys(0) = start(1);
    ss(0) = 0.00;
    ROS_WARN("Goal position %.2f, %.2f", GPR.getGoal()(0), GPR.getGoal()(1));
    ROS_WARN("Start position %.2f, %.2f", start(0), start(1));
    std::deque<Eigen::Vector2d> temp;
    
    for (int i=1; i < N; ++i){
      ROS_WARN("IN LOOP");
      ctr++;
      double dist_to_goal = (GPR.getGoal() - current_pt).squaredNorm();
      if (i%2==0){ 
        temp.push_back(current_pt);
      } else{
        temp.push_front(current_pt);
      }
      if (dist_to_goal < 0.1) {
        break;
        ROS_WARN("REACHED GOAL");
      }

      Eigen::Vector2d front = temp.front();
      Eigen::Vector2d back = temp.back();

      Eigen::Vector2d grad = GPR.getTotalGradient(current_pt);

      double grad_norm = grad.norm();

      if (grad_norm < 1e-6){
        ROS_WARN("grad norm is low");
        break;
      }
      
  //		grad /= grad_norm;
      double step = std::clamp(grad_norm, 0.0, resolution);   // e.g. 0.02 – 0.10 m
      current_pt += step * grad / grad_norm;                // same as “dt · v”
  //		current_pt += resolution * grad;

      if (front.isApprox(current_pt, 1e-2) || back.isApprox(current_pt, 1e-2)) {
        ROS_WARN("Oscillating");
        break;
      }
      if (temp.size() > 2) temp.pop_back(); 

      xs(i) = current_pt(0);	
      ys(i) = current_pt(1);

      double dx = (xs)(i) - (xs)(i-1);
      double dy = (ys)(i) - (ys)(i-1);
      (ss)(i) = (ss)(i-1) + std::hypot(dx,dy);


      ROS_WARN("[step %3d] s=%.3f  pos=(%.3f, %.3f)  "
                   "grad=(%.3f, %.3f) |∇U|=%.3f  dist_goal=%.3f",
                   i,
                   ss(i-1),          // arc-length up to previous point
                   current_pt.x(),
                   current_pt.y(),
                   grad.x(),
                   grad.y(),
                   grad_norm,
                   dist_to_goal);
    }
    ROS_WARN("i value %d", ctr);
    const int n_pts = ctr;  
    xs.conservativeResize(n_pts);
    ys.conservativeResize(n_pts);	
    ss.conservativeResize(n_pts);
  }

	};
}
