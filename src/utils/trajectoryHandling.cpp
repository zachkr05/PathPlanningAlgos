
#include "utils/trajectoryHandling.hpp"



void Utils::generateClampedBSpline(const std::Vector<Eigen::Vector2d> ctrl_pts, int totalSamples, int degree = 3, std::vector<Eigen::Vector2d>& currTraj){


  std::vector<Eigen::Vector2d> out;
  std::vector<double> K=generateClampedKnots(ctrl.size(), degree);

  const double tMin = K[degree];
  const double tMax = K[K.size() -1 - degree];

  out.reserve(totalSamples);
  for (int i = 0; i<totalSamples;++i){

    double u = static_cast<double>(i) / (totalSamples-1); //normalize the parameter to be between [0,1]
    double t = tMin+u*(tMax-tMin); //map the point to be inside of the curve
    out.emplace_back(Utils::deBoor(t,degree,ctrl_pts,K)); //evaluate the constructed Bsplinbe at point t

  }

  currTraj=out;

}


Eigen::Vector2d deBoor(double t, int degree, const std::vector<Eigen::Vector2d>& P, const std::vector<double>& K){



}

//https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve.html
// NOTE: m is an index not a count.
// m = n + p + 2
// p = degree
// n = index of control pts
// m = index of the last knot
void Utils::generateClampedKnots(int nPts, int degree){

  //so the number of knots (m) 
  const int m = nPts + degree + 1;

  //store the knots
  std::vector<double> K(m, 0.0);

  //Find all the internal knots
  for (int i = degree +1; i<nPts+1; ++i)
    K[i] = static_cast<double>(i-degree);

  // 'clamp' the B-Spline i.e the final four pts must be the end value such that it interpolates to the final pt
  // https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html
    // Unlike Bézier curves, B-splines do not generally pass through their first or last control points.
    // Increasing the multiplicity of a knot reduces continuity: a knot with multiplicity k gives C^(p-k) continuity.
    // Repeating the first and last knots (p+1) times forces the curve to interpolate the first and last control points.
    // Such knot vectors are called "clamped" and produce endpoint tangency to the control polygon.
    
  const double last = static_cast<double>(nPts-degree+1);
  for (int i = nPts +1; i<m; ++i)
    K[i] = last;

  return K;

}



