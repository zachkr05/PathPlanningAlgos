// robot_tracking.hpp
// -----------------------------------------------------------------------------
// Header‑only, ultra‑lightweight robot tracker that keeps **only the latest
// planar (x,y) position** of the robot published on a ROS topic
// (geometry_msgs/TransformStamped).
//
//  • Storage: two `std::atomic<double>` values (x_, y_) → lock‑free, cache‑hot.
//    ‑ Reads are **wait‑free**; writes in the ROS callback are minimal cost.
//  • No dynamic allocations, no vectors, no growing memory footprint.
//  • Header‑only (all inline) → zero link overhead.
// -----------------------------------------------------------------------------

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <atomic>
#include <utility>

namespace WorldState {

class RobotTracker {
public:
    /**
     * Construct the tracker.
     * @param nh        ROS NodeHandle for subscription.
     * @param topicName Topic publishing geometry_msgs/TransformStamped
     *                  (e.g. "/vicon/robot/robot").
     */
    RobotTracker(ros::NodeHandle& nh, const std::string& topicName)
    {
        _sub = nh.subscribe(topicName, 10, &RobotTracker::callback, this);
    }

    /** Return the latest (x,y) as a std::pair<double,double>. */
    inline std::pair<double, double> position() const noexcept
    {
        return { _x.load(std::memory_order_relaxed),
                 _y.load(std::memory_order_relaxed) };
    }

private:
    // ROS callback – overwrite x_, y_ with the newest sample (z ignored)
    void callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        _x.store(msg->transform.translation.x, std::memory_order_relaxed);
        _y.store(msg->transform.translation.y, std::memory_order_relaxed);
    }

    ros::Subscriber   _sub;
    std::atomic<double> _x{0.0};
    std::atomic<double> _y{0.0};
};

}  // namespace RobotTracking

