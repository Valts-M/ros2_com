#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>
#include "ros2_com/srv/save_map.hpp"
#include <slam_toolbox/srv/pause.hpp>

//robotv3
#include <shmem/shmem_cb_consumer.hpp>
#include <robot_pose.hpp>
#include <data_structures/ros_flags.hpp>

#include "ouster/os1_datatypes.h"

namespace ros2_com
{

using namespace zbot;

class RosManager : public rclcpp::Node
{
  using Storage = boost::interprocess::managed_shared_memory;
  using ShmemFlagConsumer = shmem::ShmemCBConsumer<RosFlags, shmem::PolicyFifo, Storage>;

public:
  RosManager();
  RosManager(const rclcpp::NodeOptions& t_options);
  ~RosManager();

private:
  RosFlags m_currFlags;

  std::map<processId, pid_t> m_pidMap
  {
      {odom, 0},
      {mapping, 0},
      {localization, 0},
      {logging, 0}
  };

  std::map<processId, std::string> m_commandMap
  {
      {odom, "ros2 launch ros2_com odom.launch.py"},
      {mapping, "ros2 launch ros2_com slam.launch.py"},
      {localization, "ros2 launch ros2_com localization.launch.py"},
      {logging, "ros2 bag record -a"}
  };

  std::map<processId, int32_t> m_stopCountMap
  {
      {odom, 0},
      {mapping, 0},
      {localization, 0},
      {logging, 0}
  };

  std::map<processId, bool> m_flagMap
  {
      {odom, false},
      {mapping, false},
      {localization, false},
      {logging, false}
  };

  std::map<processId, bool> m_restartMap
  {
      {odom, false},
      {localization, false},
      {mapping, false},
      {logging, false}
  };

  bool m_saveMapFlag{false};

  rclcpp::TimerBase::SharedPtr m_rosTimer;

  rclcpp::Client<ros2_com::srv::SaveMap>::SharedPtr m_mapSaver;
  rclcpp::Client<slam_toolbox::srv::Pause>::SharedPtr m_slamPauseToggler;

  std::unique_ptr<ShmemFlagConsumer> m_flagCon
  sumer{nullptr};

  void getRosFlags();

  void saveMap();

  void updateHandler();

  void updateProcessStates();

  void updateProcessState(const processId & t_processId);

  void sendStop(const processId & t_processId);

  void startProcess(const processId & t_processId);

  bool isProcessRunning(const processId & t_processId);

  void sendKill(const processId & t_processId);

  bool incompatibleProcesses(const processId & t_processId);

  void killAll();

  /*!
    * @brief Allocates and starts the shmem smart pointers
  */
  void allocateShmem();
  /*!
    * @brief Deallocates the shmem smart pointers
  */
  void deallocateShmem();
  /*!
    * @brief Starts the shmem smart pointers
  */
  void startShmem();
  /*!
    * @brief Stops the shmem smart pointers
  */
  void stopShmem();
  /*!
    * @brief Checks if the shemem smart pointers are allocated
  */
  bool needAllocateShmem();

};
}
#endif //ROS_MANAGER_H