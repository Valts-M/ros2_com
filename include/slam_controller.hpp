#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <filesystem>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>
#include "ros2_com/srv/save_map.hpp"
#include "ros2_com/srv/reset_odom.hpp"

//robotv3
#include <robot_pose.hpp>
#include <data_structures/common_data_structures.hpp>
#include <robot_pose.hpp>

#include "shmem_util.hpp"
#include <data_structures/ros_flags.hpp>

namespace ros2_com
{

using namespace zbot;

/**
 * @brief Main node that is run at startup. Manages all the other ros nodes as subprocesses.
 * Receives flags from RobotV3 on what to turn on/off and when and makes sure to shutdown things in
 * the correct order. Will restart any nodes that crash. Is also responsible for calling services
 * for saving maps, resetting odometry, etc.
 * 
 */
class SlamController : public rclcpp::Node
{

public:
/**
 * @brief Construct a new SlamController object
 * 
 */
  SlamController();

  /**
   * @brief Destroy the SlamController object
   * 
   */
  ~SlamController();

private:

  /**
   * @brief Directory where all slam maps are saved
   * 
   */
  const std::filesystem::path m_slamMapsDir{"/home/RobotV3/slam_maps"};
  /**
   * @brief Path to the latest created map
   * 
   */
  std::filesystem::path m_latestMapPath;

  /**
   * @brief Shmem utility
   */
  std::unique_ptr<ShmemUtility> m_shmemUtil;

  pid_t slamPID{0};

  SlamMode slamMode; 

  /**
   * @brief Timer that calles the update handler at a given interval
   * 
   */
  rclcpp::TimerBase::SharedPtr m_rosTimer;

  /**
   * @brief Client for the map saver service
   * 
   */
  rclcpp::Client<ros2_com::srv::SaveMap>::SharedPtr m_mapSaver;

  void createMapSaveDirStructure();

  void createServerMapDir();

  void startSlam();

  void stopSlam();

  bool getSlamMode();

  bool isSlamRunning();

  /**
   * @brief Calls the save map service as well as receives the reply
   * 
   */
  void saveMap();

  /**
   * @brief Creates a folder for where to store the map files and returns the path
   * of where to save the map
   * 
   */
  std::string createMapSavePath();

  /**
   * @brief Reads the num.txt file in the map save path and returns a valid path to the latest
   * created map
   * 
   */
  std::filesystem::path getLatestMapPath();

  /**
   * @brief Update handler called by the ros timer every n ms
   * 
   */
  void updateHandler();

  bool getMapFromServer();

};
}
#endif //ROS_MANAGER_H