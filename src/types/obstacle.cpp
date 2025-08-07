
#include "types/obstacle.hpp"
#include <algorithm>
#include <cmath>


namespace Environment{

  /**
   * @brief Computes the gradient for a given point.
   *
   * The gradient consists of a repulsive component (pointing outward from the obstacle)
   * and a rotational component (tangent to the obstacle surface) to encourage moving around it.
   *
   * References:
   *   - https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
   *   - https://ieeexplore.ieee.org/document/10156492
   *
   * @param point The point at which to evaluate the gradient.
   * @param goal_pos The current position of the goal.
   * @return The gradient vector at the specified point.
   */

  const Eigen::Vector2d& Obstacle::getGradient(Eigen::Vector2d point, const Eigen::Vector2d& goal_pos) const noexcept{

    double d = std::max(1e-6, dist(point));
    Eigen::Vector2d normal_vec = normal(point);

    double ptl_mag = amplitude * std::exp(-0.5 * d * d / (sigma * sigma));

    // Tangent vector (orthogonal to the normal): used for rotational force field
    // Matches F_rot = [-Fy, Fx] as in:
    // A. Srivastava et al., "A Modified Artificial Potential Field for UAV Collision Avoidance", ICUAS 2023. DOI: 10.1109/ICUAS57906.2023.10156492
    Eigen::Vector2d grad_repulsive = ptl_mag * normal_vec / (sigma*sigma);

    // Tangent vector (orthogonal to the normal): used for rotational force field
    // Matches F_rot = [-Fy, Fx] as in:
    // A. Srivastava et al., "A Modified Artificial Potential Field for UAV Collision Avoidance", ICUAS 2023. DOI: 10.1109/ICUAS57906.2023.10156492
    Eigen::Vector2d tangent_vec(-normal_vec.y(), normal_vec.x());

    // Direction from current point to goal (normalized)
    Eigen::Vector2d goal_dir = (goal_pos - point).normalized();


    // Determine direction of rotation (clockwise/counterclockwise)
    // Equivalent to sign of cross(normal, goal_dir) — matches Equation (7) in Srivastava et al. (2023)
    double rot_sign = (normal_vec.x() * goal_dir.y() - normal_vec.y() * goal_dir.x() > 0 ? 1.0 : -1.0);

    // Rotational gradient: encourages navigation around obstacle tangentially
    // Inspired by rotational field in [Srivastava et al., 2023, Eq. (5–7)]
    double k_rot  = 0.8;
    Eigen::Vector2d grad_rot = k_rot * rot_sign * ptl_mag * tangent_vec;

    return grad_repulsive + grad_rot;                 

  }


  /**
   * @brief Computes the potential at a given point due to this obstacle.
   *
   * The potential is calculated using a Gaussian function of the signed distance from the obstacle's surface.
   *
   * @param point The point at which to evaluate the potential.
   * @return The potential value at the specified point.
   */

  double Obstacle::getPotential(Eigen::Vector2d& point) const noexcept{
    double d = std::max(1e-6, dist(point));
    return amplitude * std::exp(-0.5 * d * d / (sigma * sigma));
  }


  /**
   * @brief Computes the outward normal vector at a given point relative to the obstacle center.
   *
   * @param p The point at which to compute the normal.
   * @return The normalized outward vector from the obstacle center to the point, or zero if the point coincides with the center.
   */

  Eigen::Vector2d Obstacle::normal(const Eigen::Vector2d& p) const noexcept {
    Eigen::Vector2d n = (p - center);
    double nrm = n.norm();
    return n.squaredNorm() > 1e-18 ? n.normalized() : Eigen::Vector2d::Zero();
  }
}

