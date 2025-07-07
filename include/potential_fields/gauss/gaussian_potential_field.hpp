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

	bool generateTrajectory();

	void planTrajectory();
	


	};
}
