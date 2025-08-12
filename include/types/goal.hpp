#pragma once

#include <string>
#include <cmath>
#include <Eigen/Dense>
#include "types/goal.hpp"
namespace World{
    struct Goal{
		Eigen::Vector2d position {0.0, 0.0};
		double k_att {1.0};
		};
}
