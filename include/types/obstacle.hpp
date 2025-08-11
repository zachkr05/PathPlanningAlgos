#pragma once
#include <string>
#include <cmath>
#include <Eigen/Dense>


namespace Environment {
	struct Obstacle {
			std::string id;
			Eigen::Vector2d center {0.0, 0.0};
			double radius {5};
		};
}
