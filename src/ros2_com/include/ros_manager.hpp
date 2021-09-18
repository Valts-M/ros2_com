#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"

//robotv3
#include <shmem/shmem_position_producer.hpp>
#include <robot_pose.hpp>

namespace ros2_com
{

using namespace zbot;

class RosManager : public rclcpp::Node
{

  using Storage = boost::interprocess::managed_shared_memory;
  using ShmemFlagConsumer = shmem::ShmemPositionProducer<Storage>;

public:
  RosManager();
  RosManager(const rclcpp::NodeOptions& t_options);
  ~RosManager();

private:

  rclcpp::TimerBase::SharedPtr m_rosTimer;

  void updateHandler();

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