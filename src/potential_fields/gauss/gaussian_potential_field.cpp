


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
using Point = Eigen::Vector2d;
using namespace PotentialFields;
 using Obstacle = Environment::Obstacle;
    using Map = std::map<UserPreferences::PotentialFieldParams, std::vector<Obstacle>>;
    using Preferences = UserPreferences::PotentialFieldParams;
   using Goal = World::Goal;




const std::vector<Eigen::Vector2d> GPF::generateTrajectory(Eigen::Vector2d& pos_r, Map& obstacles, Goal& goal){


  Eigen::Vector2d current_pt = pos_r;

  std::vector<Eigen::Vector2d> trajectory;

  for(int i = 0; i < MAX_PTS; ++i){

    //get the total gradient amongst all obstacles
    
    const Eigen::Vector2d grad = this->calculateTotalGradient(current_pt, obstacles, goal);
    const double grad_norm = grad.norm();


    double step = std::clamp(grad_norm, 0.0, RESOLUTION);
    current_pt += step*grad / grad_norm;

    trajectory.push_back(current_pt);
  }

  return trajectory;
}

const Eigen::Vector2d GPF::calculateAttractiveGradient(Eigen::Vector2d& point, Goal& goal){
  return goal.getAttractiveGradient(point);
}

const Eigen::Vector2d GPF::calculateTotalGradient(Eigen::Vector2d& point, Map& obstacles,  Goal& goal){


  //Calculate the gradient from the goal
  const Eigen::Vector2d grad_attr = this->calculateAttractiveGradient(point, goal);

  //Calculate the repulsive gradient
  const Eigen::Vector2d grad_repulsive = calculateSummedRepulsiveGradient(point, obstacles);
  
  return grad_attr + grad_repulsive;
}

const Eigen::Vector2d GPF::calculateSummedRepulsiveGradient(Eigen::Vector2d& point, Map& obstacles){

  std::vector<Eigen::Vector2d> results;
  results.reserve(obstacles.size());
  size_t workers = std::thread::hardware_concurrency() / 2;
  size_t total = obstacles.size();
  size_t batch_size = (total+workers-1) / workers;

  auto it = obstacles.begin();
  for(size_t batch=0; batch<workers && it != obstacles.end(); ++batch){


    std::vector<std::future<Point>> futures;

    for (size_t count = 0; count < batch_size && it != obstacles.end(); ++count, ++it) {

      const auto& params = it->first;
      const auto& obs_list = it->second;

      futures.push_back(std::async(std::launch::async, &PotentialFields::GPF::calculateRepulsiveGradient,this, std::ref(point), std::cref(params), std::cref(obs_list)));

    }

    for (auto& f : futures){


      results.push_back((f.get()));

    }
  }

  Point response = Eigen::Vector2d::Zero();
  for (auto& res : results){

    response+= res;
  }


  return response;
}


//Calculate distance from one point to the edge of an obstacle
const double GPF::calculateSignedDistance(const Eigen::Vector2d& p,const Obstacle& obstacle) const noexcept{
  return (p-obstacle.center).norm() - obstacle.radius;
}

const Eigen::Vector2d GPF::calculateNormalVector(const Eigen::Vector2d& p, const Obstacle& obstacle) const noexcept{

  Eigen::Vector2d distance = (p-obstacle.center);
  double norm = distance.norm();
  return distance.squaredNorm() > 1e-18 ? distance.normalized() : Eigen::Vector2d::Zero();
}


Point GPF::calculateRepulsiveGradient(Eigen::Vector2d& point, const Preferences& params, const std::vector<Obstacle>& obstacle){

  double signed_dist;
  Point normal_vec;
  double ptl_mag;
  Point grad_repulsive;
  Point goal_dir;
  double rot_sign;
  Eigen::Vector2d result = Eigen::Vector2d::Zero();
  double k_rot=0.8;

  Point grad_rot;
  
  for(auto& obs : obstacle){
    signed_dist = std::max(1e-6, this->calculateSignedDistance(point, obs));
  
    normal_vec = this->calculateNormalVector(point,obs);

    ptl_mag = params.amplitude * std::exp(-0.5*signed_dist*signed_dist / (params.sigma * params.sigma));

    grad_repulsive = ptl_mag * normal_vec / (params.sigma*params.sigma);

    Eigen::Vector2d tangent_vec (-normal_vec.y(), normal_vec.x());

    goal_dir = (tangent_vec - point).normalized(); //this line needs to be fixed

    rot_sign = (normal_vec.x() * goal_dir.y() - normal_vec.y() * goal_dir.x() > 0 ? 1.0 : -1.0);

    grad_rot = k_rot * rot_sign * ptl_mag * tangent_vec;

    result += grad_repulsive + grad_rot;

  }

  return result;

}





int main() {

  GPF potential_field; 

  Eigen::Vector2d start_pos(2,2);

  World::Goal goal;

  goal.center = Eigen::Vector2d(5.0)

  
}
