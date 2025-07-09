#include <ros/ros.h>
#include "tracking/obstacle_manager.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_tracker_node");
    ros::NodeHandle nh("~");                     

    double scan_hz;
    nh.param("scan_hz", scan_hz, 1.0);           
    WorldState::start(nh, scan_hz);            

    ROS_INFO_STREAM("[ObstacleTracker] scanning ROS master at " << scan_hz << " Hz");
    ros::spin();                                 
    return 0;
}

