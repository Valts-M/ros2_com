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
#include "ros2_com/srv/save_initial_pose.hpp"
#include "ros2_com/srv/pause_pose_send.hpp"

//robotv3
#include <robot_pose.hpp>
#include "shmem_util.hpp"

namespace ros2_com
{

using namespace zbot;

/**
 * @brief Node that listens for the map -> laser_sensor_frame and odom -> laser_sensor_frame
 * transforms and sends them over Shmem to RobotV3
 * 
 */
class PoseListener : public rclcpp::Node
{

public:
/**
 * @brief Construct a new Pose Listener object
 * 
 */
  PoseListener();
  /**
   * @brief Destroy the Pose Listener object
   * 
   */
  ~PoseListener();

private:

  /**
   * @brief Send pose service pointer
   * 
   */
  rclcpp::Service<ros2_com::srv::SendInitialPose>::SharedPtr m_sendPoseService{nullptr};
  /**
   * @brief Save pose service pointer
   * 
   */
  rclcpp::Service<ros2_com::srv::SaveInitialPose>::SharedPtr m_savePoseService{nullptr};
  /**
   * @brief Initial pose publisher
   * 
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPosePublisher{nullptr};

  /**
   * @brief Geometry msg that stores the last pose from SLAM or 0 0 0
   * 
   */
  geometry_msgs::msg::PoseWithCovarianceStamped m_initialPose{};

  /**
   * @brief Stores the odom->laser_sensor_frame pose
   * 
   */
  RobotPose m_odomPose;
    /**
   * @brief Stores the map->laser_sensor_frame pose
   * 
   */
  RobotPose m_mapPose;
  /**
   * @brief Shmem utility
   * 
   */
  std::unique_ptr<ShmemUtility> m_shmemUtil;

  /**
   * @brief Callback for the Send initial pose service. Will publish the stored m_initialPose
   * to the /initialpose topic
   * 
   * @param request Pointer to service request
   * @param response Pointer to service response
   */
  void sendInitialPose(const std::shared_ptr<ros2_com::srv::SendInitialPose::Request> request,
          std::shared_ptr<ros2_com::srv::SendInitialPose::Response> response);

  /**
   * @brief Callback for the save initial pose service. Will locally store the current 
   * map -> laser_sensor_frame transform to be published later.
   * 
   * @param request Pointer to service request
   * @param response Pointer to service response
   */
  void saveInitialPose(const std::shared_ptr<ros2_com::srv::SaveInitialPose::Request> request,
          std::shared_ptr<ros2_com::srv::SaveInitialPose::Response> response);

  void pausePoseSend(const std::shared_ptr<ros2_com::srv::PausePoseSend::Request> request,
        std::shared_ptr<ros2_com::srv::PausePoseSend::Response> response);

  /**
   * @brief Ros timer callback
   * 
   */
  void timerCallback();

  /*!
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

  bool m_pausePoseSend{false};

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

  /**
   * @brief Ros timer that calls the timer callback every n ms
   * 
   */
  rclcpp::TimerBase::SharedPtr m_timer{nullptr};

  /**
   * @brief Buffer that stores transforms
   * 
   */
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

  /**
   * @brief string with the map frame name
   * 
   */
  std::string m_map_frame;

  /**
   * @brief string with the odom frame name
   * 
   */
  std::string m_odom_frame;

  /**
   * @brief string with the target frame name
   * 
   */
  std::string m_target_frame;

};
}
#endif //POSE_LISTENER_H