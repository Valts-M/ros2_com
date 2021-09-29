#ifndef POINT_2_BLOCK_H
#define POINT_2_BLOCK_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

//robotv3
#include <shmem/shmem_position_producer.hpp>
#include <robot_pose.hpp>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

namespace ros2_com
{

using namespace zbot;

class Point2Block : public rclcpp::Node
{

  using Storage = boost::interprocess::managed_shared_memory;
  using ShmemPoseProducer = shmem::ShmemPositionProducer<Storage>;

public:
  Point2Block();

private:

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscriber{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher{nullptr};
  std::unique_ptr<ShmemPoseProducer> m_odomPoseProducer{nullptr};

  void topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /*!
    * @brief Stores how many ros messages have been published
  */
  size_t m_count;

  /*!
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

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
#endif //POINT_2_BLOCK_H