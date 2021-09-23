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

//robotv3
#include <shmem/shmem_cb_consumer.hpp>
#include <shmem/shmem_raw_producer.hpp>
#include <robot_pose.hpp>
#include <data_structures/common_data_structures.hpp>

#include "ros_manager_maps.hpp"

namespace ros2_com
{

using namespace zbot;

class RosManager : public rclcpp::Node
{
  using Storage = boost::interprocess::managed_shared_memory;
  using ShmemFlagConsumer = shmem::ShmemCBConsumer<RosFlags, shmem::PolicyFifo, Storage>;
  using ShmemSlamMapPathProducer = shmem::ShmemRawProducer<TextualInfo, Storage>;

public:
  RosManager();
  RosManager(const rclcpp::NodeOptions& t_options);
  ~RosManager();

private:

  const std::string m_mapSavePath{"/home/RobotV3/slam_maps"};
  const TextualInfo m_text{m_mapSavePath.c_str()};

  RosFlags m_latestFlags;

  bool m_saveMapFlag{false};
  bool m_mapSavePending{false};

  rclcpp::TimerBase::SharedPtr m_rosTimer;

  rclcpp::Client<ros2_com::srv::SaveMap>::SharedPtr m_mapSaver;

  std::unique_ptr<ShmemFlagConsumer> m_flagConsumer{nullptr};

  std::unique_ptr<ShmemSlamMapPathProducer> m_slamPathProducer{nullptr};

  bool getRosFlags();

  void setLocalFlags();

  void setStateFlag(const processId & t_processId);

  void saveMap();

  std::string createMapSavePath();

  void updateHandler();

  void updateProcessStates();

  void updateProcessState(const processId & t_processId);

  void startProcess(const processId & t_processId);

  void stopProcess(const processId & t_processId);

  void sendStop(const processId & t_processId);

  void sendKill(const processId & t_processId);

  void stopAll();

  bool isProcessRunning(const processId & t_processId);

  bool incompatibleProcesses(const processId & t_processId);

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