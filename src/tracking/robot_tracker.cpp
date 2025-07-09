#include <ros/ros.h>
#include "tracking/robot_tracker.hpp"   // header-only class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tracker_node");
    ros::NodeHandle nh("~");                       // private namespace

    // Allow topic to be set via a ROS parameter; default to Vicon robot topic.
    std::string topic;
    nh.param<std::string>("topic", topic, "/vicon/robot/robot");

    WorldState::RobotTracker tracker(nh, topic);

    ROS_INFO_STREAM("RobotTracker: listening on " << topic);
    ros::spin();                                   // single-threaded callback loop
    return 0;
}

