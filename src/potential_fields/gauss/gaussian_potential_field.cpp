
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

using namespace PotentialFields;

/*
 *
 * @brief Generates a trajectory using the Gaussian Potential Field
 *
 * @param[in] pos_r The position of the robot
 * @param[in] obstacles map of <learned params -> list of obstacles>
 * @param[in] goal the target/end point for the trajectory
 *
 * @return returns a full trajectory as a list of points
 */

const std::vector<Eigen::Vector2d> GPF::generateTrajectory(const Point& pos_r, const Map& obstacles, const Goal& goal){

  //Initialize values
  Point current_pt = pos_r;
  std::vector<Point> trajectory;

  Point grad;
  double grad_norm;

  //Calculate the trajectory point by point
  for(int i = 0; i < MAX_PTS; ++i){

    //Calculate the gradient
    grad = this->calculateTotalGradient(current_pt, obstacles, goal);
    grad_norm = grad.norm();
    double step = std::clamp(grad_norm, 0.0, RESOLUTION);

    current_pt += step*grad / grad_norm;
    trajectory.push_back(current_pt);

  }

  return trajectory;

}


/*
 *
 * @brief Calculates the attractive gradient
 *
 * @param[in] point the point in the world space in which to calculate the attractive gradient
 * @param[in] goal the target/end point for the trajectory
 *
 * @return returns a 2d vector of the x and y components of the attractive force
 *
 */ 

const Point GPF::calculateAttractiveGradient(const Point& point, const Goal& goal){
  return -goal.k_att * (point - goal.position);
}

/*
 *
 * @brief calculates the combined gradient [ (gradient of attractive force) + (gradient of repulsive force)]
 *
 * @param[in] point the point at which to evaluate the gradient
 * @param[in] obstacles map of <learned params -> list of obstacles>
 * @param[in] goal the target/end point for the trajectory
 *
 * @return returns the combined gradient with x and y components. 
 *
 */

const Point GPF::calculateTotalGradient(const Point& point, const Map& obstacles, const Goal& goal){

  //Calculate the gradient from the goal
  const Eigen::Vector2d grad_attr = this->calculateAttractiveGradient(point, goal);

  //Calculate the repulsive gradient
  const Eigen::Vector2d grad_repulsive = calculateSummedRepulsiveGradient(point, obstacles,goal);
  
  return grad_attr + grad_repulsive;
}



//note there is definitely a better way to do the threading.
//
//
//
/* @brief Calculates the repulsive gradient for summed among all obstacles
 *
 * @param[in] point the point at which to evaluate the gradient
 * @param[in] obstacles map of <learned params -> list of obstacles>
 * @param[in] goal the target/end point for the trajectory
 *
 * @return returns the x and y components of the summed gradient
 *
 */
const Point GPF::calculateSummedRepulsiveGradient(const Point& point, const Map& obstacles, const Goal& goal){

  size_t workers = std::thread::hardware_concurrency() / 2; //Safety net
  size_t total = obstacles.size(); 
  size_t batch_size = (total+workers-1) / workers;  //evenly disperse the threads
  auto it = obstacles.begin();
  Point response = Eigen::Vector2d::Zero();

  for(size_t batch=0; batch<workers && it != obstacles.end(); ++batch){

    std::vector<std::future<Point>> futures;

    // Make the threads
    for (size_t count = 0; count < batch_size && it != obstacles.end(); ++count, ++it) {

      const auto& params = it->first;
      const auto& obs_list = it->second;

      futures.push_back(std::async(std::launch::async, &PotentialFields::GPF::calculateRepulsiveGradient, this, std::ref(point), std::cref(params), std::cref(obs_list), std::cref(goal)));

    }

    //Sum over all threads 
    for (auto& f : futures){

      response += (f.get());

    }
  }


  return response;
}


//Calculate distance from one point to the edge of an obstacle
const double GPF::calculateSignedDistance(const Point& p,const Obstacle& obstacle) const noexcept{
  return (p-obstacle.center).norm() - obstacle.radius;
}

const Eigen::Vector2d GPF::calculateNormalVector(const Point& p, const Obstacle& obstacle) const noexcept{

  Point distance = (p-obstacle.center);
  double norm = distance.norm();
  return distance.squaredNorm() > 1e-18 ? distance.normalized() : Eigen::Vector2d::Zero();
}


Point GPF::calculateRepulsiveGradient(const Point& point, const Preferences& params, const std::vector<Obstacle>& obstacle, const Goal& goal){

  double signed_dist;
  Point normal_vec;
  double ptl_mag;
  Point grad_repulsive;
  Point goal_dir;
  double rot_sign;

  Point result = Eigen::Vector2d::Zero();
  double k_rot=0.8;

  Point grad_rot;
  
  for(auto& obs : obstacle){
    signed_dist = std::max(1e-6, this->calculateSignedDistance(point, obs));
  
    normal_vec = this->calculateNormalVector(point,obs);

    ptl_mag = params.amplitude * std::exp(-0.5*signed_dist*signed_dist / (params.sigma * params.sigma));

    grad_repulsive = ptl_mag * normal_vec / (params.sigma*params.sigma);

    Eigen::Vector2d tangent_vec (-normal_vec.y(), normal_vec.x());

    goal_dir = (goal.position - point).normalized(); //this line needs to be fixed

    rot_sign = (normal_vec.x() * goal_dir.y() - normal_vec.y() * goal_dir.x() > 0 ? 1.0 : -1.0);

    grad_rot = k_rot * rot_sign * ptl_mag * tangent_vec;

    result += grad_repulsive + grad_rot;

  }

  return result;

}



int main() {
    // Instantiate the potential field
    GPF potential_field;

    // Start position
    Eigen::Vector2d start_pos(0.0, 0.0);

    // Create a goal at (5,5)
    World::Goal goal;
    Eigen::Vector2d goal_pos(5.0, 5.0);
    goal.position = goal_pos;

    // Create obstacle parameters
    UserPreferences::PotentialFieldParams params1;
    params1.id = "set1";

    // Create obstacles
    Environment::Obstacle obstacle1;
    obstacle1.id = "obs1";
    obstacle1.center = Eigen::Vector2d(2.0, 0.0);

    Environment::Obstacle obstacle2;
    obstacle2.id = "obs2";
    obstacle2.center = Eigen::Vector2d(0.0, 2.0);

    // Map of params -> list of obstacles
    Map obstacles;
    obstacles[params1] = { obstacle1, obstacle2 };

    // Generate trajectory
    auto trajectory = potential_field.generateTrajectory(start_pos, obstacles, goal);

    // Print the trajectory
    std::cout << "Generated trajectory (" << trajectory.size() << " points):\n";
    for (size_t i = 0; i < trajectory.size(); ++i) {
        std::cout << i << ": (" 
                  << trajectory[i].x() << ", " 
                  << trajectory[i].y() << ")\n";
    }

    return 0;
}
