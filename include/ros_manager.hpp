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
#include "ros2_com/srv/send_initial_pose.hpp"
#include "ros2_com/srv/save_initial_pose.hpp"

//robotv3
#include <robot_pose.hpp>
#include <data_structures/common_data_structures.hpp>
#include <robot_pose.hpp>

#include "ros_manager_maps.hpp"
#include "shmem_util.hpp"

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
class RosManager : public rclcpp::Node
{

public:
/**
 * @brief Construct a new Ros Manager object
 * 
 */
  RosManager();
  /**
   * @brief Construct a new Ros Manager object
   * 
   * @param t_options 
   */
  RosManager(const rclcpp::NodeOptions& t_options);
  /**
   * @brief Destroy the Ros Manager object
   * 
   */
  ~RosManager();

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
   * @brief Latest flag message received from RobotV3
   * 
   */
  RosFlags m_latestFlags;
  /**
   * @brief Shmem utility
   * 
   */
  std::unique_ptr<ShmemUtility> m_shmemUtil;

  /**
   * @brief Flag that signals that a map should be saved
   * 
   */
  bool m_saveMapFlag{false};
  /**
   * @brief Flag that signals that a response from the map saver service is pending
   * 
   */
  bool m_mapSavePending{false};

  /**
   * @brief Flag that signals that the last mapping pose should be saved 
   * for switching to localization
   * 
   */
  bool m_saveInitialPose{false};
  /**
   * @brief Flag that signals that a response from the save initial pose service is pending
   * 
   */
  bool m_saveInitialPosePending{false};

  /**
   * @brief Flag that signals that the initial pose should be published on the /initialpose topic
   * for the localization
   * 
   */
  bool m_sendInitialPose{false};
  /**
   * @brief Flag that signals that a response from the send initial pose service is pending
   * 
   */
  bool m_sendInitialPosePending{false};

  /**
   * @brief Flag that signals that the odometry should be reset
   * 
   */
  bool m_resetOdomFlag{false};

  RobotPose m_initialPose{0.0, 0.0, 0.0};

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
  /**
   * @brief Client for the odom resetter service
   * 
   */
  rclcpp::Client<ros2_com::srv::ResetOdom>::SharedPtr m_odomResetter;
  /**
   * @brief Client for the initial pose sender service
   * 
   */
  rclcpp::Client<ros2_com::srv::SendInitialPose>::SharedPtr m_initialPoseSender;
  /**
   * @brief Client for the initial pose saver service
   * 
   */
  rclcpp::Client<ros2_com::srv::SaveInitialPose>::SharedPtr m_initialPoseSaver;

  /**
   * @brief Polls Shmem to see if any ros flags have been sent
   * 
   * @return true if received ros flags
   * @return false if no ros flags were received
   */
  bool getRosFlags();

  /**
   * @brief Loops through all the processes and sets their flags according to what 
   * was received from RobotV3
   * 
   */
  void setLocalFlags();

  /**
   * @brief Sets the appropriate flags depending on what process gets turned on or off
   * 
   * @param t_processId 
   */
  void setStateFlag(const processId & t_processId);

  /**
   * @brief Calls the save map service as well as receives the reply
   * 
   */
  void saveMap();

  /**
   * @brief Creates a folder for where to store the map files and returns the path
   * of where to save the map
   * 
   * @return std::string map save path
   */
  std::string createMapSavePath();

  /**
   * @brief Reads the num.txt file in the map save path and returns a valid path to the latest
   * created map
   * 
   * @return std::string map path
   */
  std::filesystem::path initLatestMapPath();

  /**
   * @brief Update handler called by the ros timer every n ms
   * 
   */
  void updateHandler();

  /**
   * @brief Loops through each process and updates it's state
   * 
   */
  void updateProcessStates();

  /**
   * @brief Updates the given processes state
   * 
   * @param t_processId 
   */
  void updateProcessState(const processId & t_processId);

  /**
   * @brief If the process is not already running, turns it on
   * 
   * @param t_processId 
   */
  void startProcess(const processId & t_processId);

  /**
   * @brief If the process is running tries to turn it off for a time. If the process
   * doesn't stop within the timeout time, kills the process.
   * 
   * @param t_processId 
   */
  void stopProcess(const processId & t_processId);

  /**
   * @brief Sends SIG_INT to the process
   * 
   * @param t_processId 
   */
  void sendStop(const processId & t_processId);

  /**
   * @brief Sends SIG_KILL to the process
   * 
   * @param t_processId 
   */
  void sendKill(const processId & t_processId);

  /**
   * @brief Sends SIG_INT to all running processess
   * 
   */
  void stopAll();

  /**
   * @brief Calls the reset odom service
   * 
   */
  void resetOdom();

  /**
   * @brief Calls the send initial pose service
   * 
   */
  void sendInitialPose();

  /**
   * @brief Calls the save initial pose service
   * 
   */
  void saveInitialPose();

  /**
   * @brief Sets the appropriate flags for turning off mapping
   * 
   */
  void turnOffMapping();

  /**
   * @brief Checks if the given process is running
   * 
   * @param t_processId 
   * @return true if running
   * @return false if not running
   * 
   * @details First checks if the processId has a valid PID associated to it. If it does then
   * sends the 0 signal to it, to check if it is alive
   */
  bool isProcessRunning(const processId & t_processId);

  /**
   * @brief Checks if there is a call to turn on mapping or localization while the other is active.
   * If true sets the flag for the other process to turn off and returns true
   * 
   * @param t_processId 
   * @return true if incompatible processes
   * @return false otherwise
   */
  bool incompatibleProcesses(const processId & t_processId);

};
}
#endif //ROS_MANAGER_H