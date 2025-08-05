
#include "tracking/obstacle_manager.hpp"
#include "types/obstacle.hpp"
#include "utils/traj_utils.hpp"
namespace WorldState {

  class ObstacleManager{


    public:

    /**
     * @brief Initializes periodic obstacle tracking using a ROS timer.
     *
     * This function sets up a ROS timer that calls the `scanTimerCb` callback
     * at a regular interval defined by the input frequency `hz`. The callback
     * is responsible for dynamically detecting and subscribing to Vicon
     * obstacle topics matching a specific naming pattern.
     *
     * @param nh A reference to the ROS NodeHandle used to create the timer and subscribers.
     * @param hz The frequency (in Hz) at which to scan for new obstacle topics. Default is 1.0 Hz.
     */
    void ObstacleManager::initializeObstacleTracking(ros::NodeHandle& nh, double hz = 1.0)
    {

      //Every update interval (1/hz) call the scan timer function
      static ros::Timer timer = nh.createTimer(
      ros::Duration(1.0 / hz),
      boost::bind(&scanTimerCb, _1, &nh));
    }


    private:

    /**
   * @brief Timer callback that scans for new Vicon obstacle topics and sets up subscriptions.
   *
   * This function is called periodically by a ROS timer. It retrieves all currently available ROS topics
   * and filters them using a regular expression that matches Vicon obstacle topics of the form:
   * `/vicon/obstacle_<num>/obstacle_<num>`. For any new matching topics not already being tracked,
   * it creates a new `Obstacle` entry and subscribes to the topic with a callback.
   *
   * @param event The ROS timer event (unused but required by the ROS timer interface).
   * @param nh A pointer to the ROS NodeHandle used to create the topic subscribers.
   *
   * @details
   * - Topics are matched using the regex: `^/vicon/obstacle_\\d+/obstacle_\\d+$`
   * - The last segment of the topic name (e.g., "obstacle_42") is used as the obstacle ID.
   * - Already-tracked obstacles are skipped.
   * - Subscriptions are stored in a static `subs` map to persist for the node's lifetime.
   * - Associated `Obstacle` instances are stored in a static `obstacles` map.
   */

  void ObstacleManager::scanTimerCb(const ros::TimerEvent&, ros::NodeHandle* nh)
  {
    //get all topics
    std::vector<ros::master::TopicInfo> list;
    if (!ros::master::getTopics(list)) {
    ROS_ERROR("[obstacle_manager.hpp] Line #%d: No topics found (ros::master::getTopics failed)", __LINE__);
    return;
    }

    static const std::regex pattern("^/vicon/obstacle_\\d+/obstacle_\\d+$"); 

    for (const auto& t : list)
    {
      if (!std::regex_match(t.name, pattern)) continue;

      // Extract the substring after the right most '/' in the topic name (e.g., "obstacle_42")
      std::string id = t.name.substr(t.name.rfind('/') + 1);
      if (obstacles.count(id)) continue;           // if it is already being tracked

      //Add the current obstacle
      Obstacle obs;  
      obs.id = id;
      obstacles[id] = obs;

      // Won't resub every iteration because of the check that if it already exists
      subs[id] = nh->subscribe<geometry_msgs::TransformStamped>(
      t.name, 10,
      boost::bind(&obstacleCb, id, _1));

      ROS_INFO_STREAM("ViconTracker: now tracking " << t.name);
    }
  }


  void ObstacleManager::obstacleCb(const std::string& id,
                                  const geometry_msgs::TransformStamped::ConstPtr& msg)
  {
      std::lock_guard<std::mutex> lk(mtx_);                 // if callbacks are concurrent
      auto& obs = obstacles_[id];                           // creates if missing
      obs.setPosition(Utils::PathPlanning::to2D(*msg));     // note: *msg and to2D (not to2d)
  }




  }



  

 


  


}