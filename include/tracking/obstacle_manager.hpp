#pragma once
/**
 * @file obstacle_manager.hpp
 * @brief Class for tracking and querying dynamic obstacles in the environment.
 *
 * Typical usage:
 * @code
 *   ros::NodeHandle nh("~");
 *   Environment::ObstacleManager mgr(nh, 5.0); // 5 Hz scan
 *   ros::spin();
 *
 *   if (mgr.hasObstacle("chair_01")) {
 *       if (const Obstacle* obs = mgr.getObstacle("chair_01")) {
 *           // use *obs
 *       }
 *   }
 * @endcode
 */

#include <map>
#include <string>
#include <cstddef>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include "potential_fields/gauss/goal_obstacle.hpp"  // defines Obstacle

namespace Environment {

/**
 * @brief Manages subscriptions, periodic scans, and an in-memory registry of obstacles.
 *
 * This class encapsulates ROS resources (subscribers/timers) and a thread-safe
 * container of currently tracked obstacles. It is suitable for use as a node or
 * nodelet component.
 */
class ObstacleManager {
public:
  /// Map type used for obstacle storage.
  using Map = std::map<std::string, Obstacle>;

  /**
   * @brief Construct the manager and start periodic scanning.
   *
   * The constructor may create a timer and set up subscriptions; pass a private
   * or global node handle depending on your namespacing needs.
   *
   * @param nh Node handle used for parameters, subscribers, and timers.
   * @param hz Desired scan/update frequency in Hertz (default: 1.0 Hz).
   */
  explicit ObstacleManager(ros::NodeHandle nh, double hz = 1.0);

  /// Default destructor.
  ~ObstacleManager() = default;

  /**
   * @brief Get a read-only view of all tracked obstacles.
   *
   * @warning The returned reference is only safe to inspect briefly. Do not
   * store it beyond immediate use; the underlying container can change in callbacks.
   *
   * @return Const reference to the internal obstacle map keyed by obstacle ID.
   */
  const Map& getObstacles() const;

  /**
   * @brief Retrieve a specific obstacle by ID.
   * @param id Obstacle identifier.
   * @return Pointer to the obstacle if found; otherwise @c nullptr.
   */
  const Obstacle* getObstacle(const std::string& id) const;

  /**
   * @brief Check whether an obstacle with the given ID is currently tracked.
   * @param id Obstacle identifier.
   * @return @c true if the obstacle exists; otherwise @c false.
   */
  bool hasObstacle(const std::string& id) const;

  /**
   * @brief Get the number of currently tracked obstacles.
   * @return Count of obstacles.
   */
  std::size_t getObstacleCount() const;

private:
  /**
   * @brief Timer callback that periodically updates tracking state.
   * @param event ROS timer event.
   */
  void scanTimerCb(const ros::TimerEvent& event);

  /**
   * @brief Subscriber callback to upsert an obstacle.
   *
   * You can bind the @p id using a lambda or `boost::bind` when creating the subscriber,
   * or derive the ID from @p msg (e.g., `child_frame_id`) inside the implementation.
   *
   * @param msg Latest transform for the obstacle.
   * @param id  Stable obstacle identifier (e.g., "chair_01").
   */
  void obstacleCb(const geometry_msgs::TransformStamped::ConstPtr& msg,
                  const std::string& id);

private:
  ros::NodeHandle _nh_;                          ///< Stored node handle.
  ros::Timer      _scan_timer;                  ///< Periodic scan/update timer.

  Map _obstacles;                               ///< Tracked obstacles by ID.
  std::map<std::string, ros::Subscriber> _subs; ///< Subscribers keyed by ID/topic.
};

} // namespace Environment
