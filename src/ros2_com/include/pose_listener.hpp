#ifndef POSE_LISTENER_H
#define POSE_LISTENER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ros2_com/srv/send_initial_pose.hpp"

//robotv3
#include <shmem/shmem_position_producer.hpp>
#include <robot_pose.hpp>

namespace ros2_com
{

using namespace zbot;

class PoseListener : public rclcpp::Node
{

  using Storage = boost::interprocess::managed_shared_memory;
  using ShmemPoseProducer = shmem::ShmemPositionProducer<Storage>;

public:
  PoseListener();
  ~PoseListener();

private:

  std::unique_ptr<ShmemPoseProducer> m_odomPoseProducer{nullptr};
  std::unique_ptr<ShmemPoseProducer> m_mapPoseProducer{nullptr};
  rclcpp::Service<ros2_com::srv::SendInitialPose>::SharedPtr m_initialPoseService{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPosePublisher{nullptr};

  RobotPose m_odomPose;
  RobotPose m_mapPose;

  void sendInitialPose(const std::shared_ptr<ros2_com::srv::SendInitialPose::Request> request,
          std::shared_ptr<ros2_com::srv::SendInitialPose::Response> response);

  void timerCallback();

  /*!
    * @brief Stores how many ros messages have been published
  */
  size_t m_count;

  /*!
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

  /*!
    * @brief Transform listener
  */
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};

  /*!
    * @brief Transform message for the odom->laser_sensor_frame transform
  */
  geometry_msgs::msg::TransformStamped m_odomLidarMsg;

  /*!
    * @brief Transform message for the map->laser_sensor_frame transform
  */
  geometry_msgs::msg::TransformStamped m_mapLidarMsg;

  rclcpp::TimerBase::SharedPtr m_timer{nullptr};

  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

  std::string m_map_frame;

  std::string m_odom_frame;

  std::string m_target_frame;

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
#endif //POSE_LISTENER_H