#pragma once
#include <string>
#include <cmath>
#include <Eigen/Dense>


namespace World {
struct Obstacle {

		std::string id;
		Eigen::Vector2d center {0.0, 0.0};
		double amplitude {1.0}; 
		double sigma {0.5}; 
		double radius {5};
	
		inline double dist(const Eigen::Vector2d& p) const noexcept {
        		return (p - center).norm() - radius;
    		}

		inline Eigen::Vector2d normal(const Eigen::Vector2d& p) const noexcept {
			Eigen::Vector2d n = (p - center);
			double nrm = n.norm();
			return n.squaredNorm() > 1e-18 ? n.normalized() : Eigen::Vector2d::Zero();
		}

		inline void setPosition(const Eigen::Vector2d& newPosition){
			center = newPosition;
		}

		inline void setParams(double newAmplitude, double newSigma){
			amplitude = newAmplitude;
			sigma = newSigma;
		}
		
		inline const double getAmplitude() const noexcept{
			return amplitude;
		}

		inline const double getSigma() const noexcept{
			return sigma;
		}

		inline const Eigen::Vector2d& getPosition() const noexcept{
			return center;
		}
		
		inline double getPotential(Eigen::Vector2d& point) const noexcept{
			double d = std::max(1e-6, dist(point));
        		return amplitude * std::exp(-0.5 * d * d / (sigma * sigma));
		}
		inline Eigen::Vector2d getGradient(Eigen::Vector2d point, const Eigen::Vector2d& goal_pos) const noexcept{


			double d = std::max(1e-6, dist(point));
			Eigen::Vector2d normal_vec = normal(point);


        		double ptl_mag = amplitude * std::exp(-0.5 * d * d / (sigma * sigma));
			
			Eigen::Vector2d grad_repulsive = ptl_mag * normal_vec / (sigma*sigma);

			Eigen::Vector2d tangent_vec(-normal_vec.y(), normal_vec.x());

			Eigen::Vector2d goal_dir = (goal_pos - point).normalized();

			double rot_sign = (normal_vec.x() * goal_dir.y() - normal_vec.y() * goal_dir.x() > 0 ? 1.0 : -1.0);

			double k_rot  = 0.8;
			Eigen::Vector2d grad_rot = k_rot * rot_sign * ptl_mag * tangent_vec;

			return grad_repulsive + grad_rot;                  // points outward

		}
		
	};
}