#pragma once
#include <string>
#include <cmath>
#include <Eigen/Dense>


namespace Environment {
	struct Obstacle {

			std::string id;
			Eigen::Vector2d center {0.0, 0.0};
			double amplitude {1.0}; 
			double sigma {0.5}; 
			double radius {5};

			/**
			 * @brief Calculates the signed distance from a point to the obstacle's surface.
			 * @param p The point to measure from.
			 * @return Signed distance: negative if inside, zero if on the surface, positive if outside.
			 */

			inline double signedDistance(const Eigen::Vector2d& p) const noexcept {
				return (p - center).norm() - radius;
			}

			/**
			 * @brief Computes the outward normal vector at a given point on the obstacle's surface.
			 * @param p The point at which to compute the normal.
			 * @return The normalized outward vector from the obstacle center to the point.
			 */

			inline Eigen::Vector2d normal(const Eigen::Vector2d& p) const

			// ===== Setters =====

			/**
			 * @brief Set the position of the obstacle center.
			 * @param newPosition The new center position.
			 */
			inline void setPosition(const Eigen::Vector2d& newPosition){
				center = newPosition;
			}

			/**
			 * @brief Set the amplitude and sigma parameters of the obstacle.
			 * @param newAmplitude The new amplitude value.
			 * @param newSigma The new sigma value.
			 */
			inline void setParams(double newAmplitude, double newSigma){
				amplitude = newAmplitude;
				sigma = newSigma;
			}

			// ===== Getters =====

			/**
			 * @brief Get the amplitude parameter of the obstacle.
			 * @return The amplitude value.
			 */
			inline const double getAmplitude() const noexcept{
				return amplitude;
			}

			/**
			 * @brief Get the sigma parameter of the obstacle.
			 * @return The sigma value.
			 */
			inline const double getSigma() const noexcept{
				return sigma;
			}

			/**
			 * @brief Get the current position of the obstacle center.
			 * @return The center position as an Eigen::Vector2d.
			 */

			inline const Eigen::Vector2d& getPosition() const noexcept{
				return center;
			}

			/**
			 * @brief Computes the potential at a given point due to this obstacle.
			 * @param point The point at which to evaluate the potential.
			 * @return The potential value at the specified point.
			 */

			double getPotential(Eigen::Vector2d& point) const noexcept;

			/**
			 * @brief Computes the gradient of the potential field at a given point.
			 * @param point The point at which to evaluate the gradient.
			 * @param goal_pos The position of the goal (may be used for direction).
			 * @return The gradient vector at the specified point.
			 */

			Eigen::Vector2d getGradient(Eigen::Vector2d point, const Eigen::Vector2d& goal_pos) const noexcept;

		}
}