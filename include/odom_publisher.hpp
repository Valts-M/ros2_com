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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"

//robotv3
#include <shmem/shmem_cb_consumer.hpp>
#include <data_structures/protocol.hpp>
#include <robot_pose.hpp>

#include "kinematics.hpp"
#include "ros2_com/srv/pause_odom.hpp"
#include "ros2_com/srv/reset_odom.hpp"
#include "shmem_util.hpp"

namespace ros2_com
{

using namespace zbot;

/**
 * @brief Node that receives encoder and gyroscope readings from Shmem and calls kinematics 
 * calculations to calculate the dead reckoning trajectory. It publishes the odom -> base_footprint
 * transform, the path taken in the odom frame, and odometry messages.
 * 
 */
class OdometryPublisher : public rclcpp::Node
{

public:
/**
 * @brief Construct a new Odometry Publisher object
 * 
 */
  OdometryPublisher();
  /**
   * @brief Construct a new Odometry Publisher object
   * 
   * @param options 
   */
  OdometryPublisher(const rclcpp::NodeOptions & options);
  /**
   * @brief Destroy the Odometry Publisher object
   * 
   */
  ~OdometryPublisher();

private:
  std::unique_ptr<ShmemUtility> m_shmemUtil;

  /**
   * @brief Gets the encoder and gyroscope scale factors from the parameter file
   * 
   * @return RobotConfig 
   */
  RobotConfig getRobotConfig();

  /**
   * @brief Initializes the odom and tf messages
   * 
   */
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
    * @brief Shared pointer to the ros odometry message publisher
  */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPublisher{nullptr};

  /*!
    * @brief Service for pausing the publishing of odometry messages
  */
  rclcpp::Service<ros2_com::srv::PauseOdom>::SharedPtr m_pauseOdomService{nullptr};

  /*!
    * @brief Service for reseting odometry to start pose (0 0 0)
  */
  rclcpp::Service<ros2_com::srv::ResetOdom>::SharedPtr m_resetOdomService{nullptr};

  /*!
    * @brief Stores how many ros messages have been published
  */
  size_t m_count;

  /*!
    * @brief Stores the PoseAndVelocity data from shared memory
  */
  MsgRawStatus m_reactdLog{};

  /**
   * @brief Flag for pausing the publishing of odom, tf and path messages
   * 
   */
  bool m_paused{false};

  /**
   * @brief Callback for the pause odom service. Flips the m_paused flag
   * 
   * @param request 
   * @param response 
   */
  void pauseToggle(const std::shared_ptr<ros2_com::srv::PauseOdom::Request> request,
          std::shared_ptr<ros2_com::srv::PauseOdom::Response> response);
  /**
   * @brief Callback for the reset odom service. Sets the current position and orientation
   * to 0 0 0 as well as clears the path vector.
   * 
   * @param request 
   * @param response 
   */
  void resetOdom(const std::shared_ptr<ros2_com::srv::ResetOdom::Request> request,
          std::shared_ptr<ros2_com::srv::ResetOdom::Response> response);

  /*!
    * @brief Stores the ros2 odometry mesage
    * @details http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  */
  nav_msgs::msg::Odometry m_odomMsg{};

  /**
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

  /**
    * @brief Timer for periodic message publishing
  */
  rclcpp::TimerBase::SharedPtr m_rosTimer{nullptr};

  /**
    * @brief Transform broadcaster for the odom->base_footprint transform
  */
  tf2_ros::TransformBroadcaster m_tfBroadcaster;

  /**
    * @brief Transform message for the odom->base_footprint transform
  */
  geometry_msgs::msg::TransformStamped m_tfMsg;

  /**
    * @brief For robot kinematics transformations calculations
  */
  Kinematics m_kinematics;

  /**
    * @brief Gets data react data from shmem
    * @return True if the data was successfully recieved, otherwise returns false
  */
  bool getPoseAndVelocity();

};
}
#endif //ODOM_PUBLISHER_H