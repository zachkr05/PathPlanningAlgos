#pragma once
// -----------------------------------------------------------------------------
// Dynamic obstacle manager WITHOUT explicit threads.
// A ros::Timer (default 1 Hz) scans the ROS master for topics matching
//     /vicon/obstacle_<N>/obstacle_<N>
// and keeps an up-to-date map of Obstacle objects.
// -----------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/TransformStamped.h>
#include <regex>
#include <map>
#include <string>
#include <Eigen/Dense>
#include "potential_fields/gauss/goal_obstacle.hpp"

namespace WorldState {

    using PathPlanning::Obstacle;

    // ------------------------- public state --------------------------------------
    inline std::map<std::string, Obstacle> obstacles;          // id → obstacle
    // -----------------------------------------------------------------------------


    // Helper: convert TransformStamped → Eigen::Vector2d (ignore z)
    inline Eigen::Vector2d to2D(const geometry_msgs::TransformStamped& m)
    {
        return { m.transform.translation.x,
                m.transform.translation.y };
    }

    // Subscriber callback (updates obstacle center)
    inline void obstacleCb(const std::string& id,
                        const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        obstacles[id].setPosition(to2D(*msg));
    }

    // Timer callback: scan master for new obstacle topics
    inline void scanTimerCb(const ros::TimerEvent&, ros::NodeHandle* nh)
    {
        std::vector<ros::master::TopicInfo> list;
        if (!ros::master::getTopics(list)) return;

        static const std::regex pat("^/vicon/obstacle_\\d+/obstacle_\\d+$");

        for (const auto& t : list)
        {
            if (!std::regex_match(t.name, pat)) continue;

            std::string id = t.name.substr(t.name.rfind('/') + 1);
            if (obstacles.count(id)) continue;           // already tracking

            Obstacle obs;  obs.id = id;
            obstacles[id] = obs;

            // create subscriber (lifetime tied to static map to stay alive)
            static std::map<std::string, ros::Subscriber> subs;
            subs[id] = nh->subscribe<geometry_msgs::TransformStamped>(
                t.name, 10,
                boost::bind(&obstacleCb, id, _1));

            ROS_INFO_STREAM("ViconTracker: now tracking " << t.name);
        }
    }

    // Initialise manager: call once from your node after ros::init()
    inline void initializeObstacleTracking(ros::NodeHandle& nh, double hz = 1.0)
    {
        static ros::Timer timer = nh.createTimer(
            ros::Duration(1.0 / hz),
            boost::bind(&scanTimerCb, _1, &nh));
    }

    // Get all obstacles (read-only access)
    inline const std::map<std::string, Obstacle>& getObstacles()
    {
        return obstacles;
    }

    // Get specific obstacle by ID (returns nullptr if not found)
    inline const Obstacle* getObstacle(const std::string& id)
    {
        auto it = obstacles.find(id);
        return (it != obstacles.end()) ? &it->second : nullptr;
    }

    // Check if obstacle exists
    inline bool hasObstacle(const std::string& id)
    {
        return obstacles.count(id) > 0;
    }

    // Get number of tracked obstacles
    inline size_t getObstacleCount()
    {
        return obstacles.size();
    }

} // namespace ViconTracker

