#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ros/ros.h>  // For ROS_WARN in implementation

namespace PathPlanning {

// Forward declaration or typedef for Spline1D
// Adjust this based on your actual spline library
using Spline1D = Eigen::Spline<double,1>; 

/**
 * @brief Smooths a trajectory using clamped B-spline interpolation
 * 
 * This function takes a trajectory defined by x,y coordinates and arc lengths,
 * converts it to a B-spline representation, and resamples it to create a 
 * smoother path.
 * 
 * @param xs Input/Output: Row vector of x-coordinates
 * @param ys Input/Output: Row vector of y-coordinates  
 * @param ss Input/Output: Row vector of arc lengths
 * @param sampleCount Number of points to sample from the B-spline (default: 500)
 */
void smoothWithClampedBSpline(Eigen::RowVectorXd& xs,
                              Eigen::RowVectorXd& ys,
                              Eigen::RowVectorXd& ss,
                              int sampleCount = 500);

// Helper functions that might be needed by the implementation
// You can make these private by not declaring them here, or public if other files need them

/**
 * @brief Removes nearly colocated points from a trajectory
 * @param points Vector of 2D points to process
 * @param eps Epsilon threshold for considering points as colocated
 */
void dedupNearlyColocated(std::vector<Eigen::Vector2d>& points, double eps = 1e-4);

/**
 * @brief Converts Catmull-Rom spline control points to B-spline control points
 * @param catmullPoints Input Catmull-Rom control points
 * @return B-spline control points
 */
std::vector<Eigen::Vector2d> catmullToBSpline(const std::vector<Eigen::Vector2d>& catmullPoints);

/**
 * @brief Softens sharp corners in a trajectory
 * @param points Control points to process (modified in place)
 */
void softenCorners(std::vector<Eigen::Vector2d>& points);

/**
 * @brief Generates points along a clamped B-spline
 * @param controlPoints B-spline control points
 * @param numSamples Number of points to generate
 * @param degree Degree of the B-spline (default: 3 for cubic)
 * @return Sampled points along the spline
 */
std::vector<Eigen::Vector2d> generateClampedBSpline(const std::vector<Eigen::Vector2d>& controlPoints,
                                                    int numSamples,
                                                    int degree = 3);

/**
 * @brief Recalculates the cumulative arc length (ss) for a given path (xs, ys).
 *
 * @param xs The X coordinates of the path.
 * @param ys The Y coordinates of the path.
 * @param ss The output cumulative arc length vector.
 */
void recalculateArcLength(const Eigen::RowVectorXd& xs,
                          const Eigen::RowVectorXd& ys,
                          Eigen::RowVectorXd& ss);

/**
 * @brief Resamples a spline trajectory by arc length
 * 
 * This function takes X and Y splines parameterized by time and resamples them
 * to be parameterized by arc length instead.
 * 
 * @param splineX Input/Output: Spline for X coordinates (will be modified)
 * @param splineY Input/Output: Spline for Y coordinates (will be modified)
 * @param start Start parameter value
 * @param end End parameter value
 * @return Total arc length of the trajectory
 */
double resampleByArcLength(Spline1D& splineX, 
                          Spline1D& splineY, 
                          double start, 
                          double end);

// Helper functions for resampleByArcLength
double compute_arclen(double start, double end, 
                     const Spline1D& splineX, 
                     const Spline1D& splineY);

double binary_search(double target_s, 
                    double t_start, 
                    double t_end, 
                    double tolerance,
                    const Spline1D& splineX, 
                    const Spline1D& splineY);

} // namespace PathPlanning