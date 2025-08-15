
#pragma once

#include <vector>
#include <Eigen/Dense>


namespace Utils {

  std::vector<Eigen::Vector2d> generateClampedBSpline(const std::Vector<Eigen::Vector2d> ctrl_pts, int totalSamples, int degree = 3);

  void catmullToBSpline(std::vector<Eigen::Vector2d>& catmull);

}
