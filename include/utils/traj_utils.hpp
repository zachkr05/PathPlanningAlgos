#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ros/ros.h>  

namespace Utils {
    namespace PathPlanning{
        inline Eigen::Vector2d to2D(const geometry_msgs::TransformStamped& msg)
        {
            return { msg.transform.translation.x,
                    msg.transform.translation.y };
        }
    }

} 