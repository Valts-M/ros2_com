#ifndef MAP_SAVER_H
#define MAP_SAVER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ros2_com/srv/save_map.hpp"

#include <shmem/shmem_cb_producer.hpp>
#include "data_structures/log_data_structures.hpp"

namespace ros2_com
{

using namespace zbot;

class MapSaver : public rclcpp::Node
{

using Storage = boost::interprocess::managed_shared_memory;
using ShmemMapProducer = shmem::ShmemCBProducer<ReactdLog, shmem::PolicyFifo, Storage>;

public:
  MapSaver();

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_subscriber;
  size_t m_count;
  rclcpp::Service<ros2_com::srv::SaveMap>::SharedPtr m_saveMapService;
  void saveMap(const std::shared_ptr<ros2_com::srv::SaveMap::Request> t_request,  
          std::shared_ptr<ros2_com::srv::SaveMap::Response> t_response);
  nav_msgs::msg::OccupancyGrid::SharedPtr m_map{nullptr};
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

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
#endif //MAP_SAVER_H