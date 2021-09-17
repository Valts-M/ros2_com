#ifndef ODOM_PUBLISHER_H
#define ODOM_PUBLISHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"

//robotv3
#include <shmem/shmem_cb_consumer.hpp>
#include "data_structures/log_data_structures.hpp"
#include <robot_pose.hpp>

#include "kinematics.h"
#include "ros2_com/srv/pause_odom.hpp"

namespace ros2_com
{

using namespace zbot;

class OdometryPublisher : public rclcpp::Node
{

  using Storage = boost::interprocess::managed_shared_memory;
  using ShmemPoseConsumer = shmem::ShmemCBConsumer<ReactdLog, shmem::PolicyFifo, Storage>;

public:
  OdometryPublisher();
  OdometryPublisher(const rclcpp::NodeOptions & options);
  ~OdometryPublisher();
  rclcpp::Context::OnShutdownCallback onShutdown();

private:

  void initMsgs();

  /*!
    * @brief Gets new reactd data and publishes tf and Odometry messages
  */
  void updateHandler();

  /*!
    * @brief Updates the transform and odometry messages
  */
  void updateOdom();

  /*!
    * @brief Updates the travelled path in the odom frame
  */
  void updatePath();

  /*!
    * @brief Shared pointer to the ros odometry message publisher
  */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPublisher{nullptr};

  /*!
    * @brief Shared pointer to the ros path message publisher
  */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pathPublisher{nullptr};

  /*!
    * @brief Service for pausing the publishing of odometry messages
  */
  rclcpp::Service<ros2_com::srv::PauseOdom>::SharedPtr m_pauseOdomService{nullptr};

  /*!
    * @brief Unique pointer to the shared memory pose consumer
  */
  std::unique_ptr<ShmemPoseConsumer> m_poseConsumer{nullptr};

  /*!
    * @brief Stores how many ros messages have been published
  */
  size_t m_count;

  /*!
    * @brief Stores the PoseAndVelocity data from shared memory
  */
  ReactdLog m_reactdLog{};

  bool m_paused{false};

  void pauseToggle(const std::shared_ptr<ros2_com::srv::PauseOdom::Request> request,
          std::shared_ptr<ros2_com::srv::PauseOdom::Response> response);

  /*!
    * @brief Stores the ros2 odometry mesage
    * @details http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  */
  nav_msgs::msg::Odometry m_odomMsg{};

  /*!
    * @brief Stores the ros2 odometry mesage
    * @details http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  */
  nav_msgs::msg::Path m_pathMsg{};

  /*!
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

  /*!
    * @brief Stores the X position of the previous message (m)
  */
  double m_previousX{0.0};

  /*!
    * @brief Stores the Y position of the previous message (m)
  */
  double m_previousY{0.0};

  /*!
    * @brief Stores the angle from the previous message (rad)
  */
  double m_previousAngle{0.0};

  /*!
    * @brief Timer for periodic message publishing
  */
  rclcpp::TimerBase::SharedPtr m_rosTimer{nullptr};

  /*!
    * @brief Transform broadcaster for the odom->base_footprint transform
  */
  tf2_ros::TransformBroadcaster m_tfBroadcaster;

  /*!
    * @brief Transform message for the odom->base_footprint transform
  */
  geometry_msgs::msg::TransformStamped m_tfMsg;

  /*!
    * @brief For robot kinematics transformations calculations
  */
  Kinematics m_kinematics;

  /*!
    * @brief Gets data from the kinematics producer
    * @return True if the data was successfully recieved, otherwise returns false
  */
  bool getPoseAndVelocity();

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
#endif //ODOM_PUBLISHER_H