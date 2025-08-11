#pragma once

#include <string>
#include <cmath>
#include <Eigen/Dense>
#include "types/goal.hpp"
namespace World{
    struct Goal{
		Eigen::Vector2d position {0.0, 0.0};
		double k_att {1.0};

		inline void setPosition(Eigen::Vector2d& newPosition) noexcept { position = newPosition;}
		inline void setAttractiveGain(double newForce) noexcept {k_att = newForce;}

		inline const double getAttractiveGain() const noexcept {return k_att;}
		inline const Eigen::Vector2d getPosition() const noexcept {return position;}

		inline const double getAttractiveForce(Eigen::Vector2d& point) const noexcept{
			return 0.5*k_att * (point-position).squaredNorm();
		}
		inline const Eigen::Vector2d getAttractiveGradient(Eigen::Vector2d point) const noexcept{
			return -k_att * (point - position);
		}

		};
}
