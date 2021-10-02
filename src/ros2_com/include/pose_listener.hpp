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

//robotv3
#include <shmem/shmem_position_producer.hpp>
#include <robot_pose.hpp>

#include "shmem_util.hpp"

namespace ros2_com
{

using namespace zbot;

class PoseListener : public rclcpp::Node
{

public:
  PoseListener();
  ~PoseListener();

private:

  rclcpp::Service<ros2_com::srv::SendInitialPose>::SharedPtr m_sendPoseService{nullptr};
  rclcpp::Service<ros2_com::srv::SaveInitialPose>::SharedPtr m_savePoseService{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPosePublisher{nullptr};

  geometry_msgs::msg::PoseWithCovarianceStamped m_initialPose{};

  RobotPose m_odomPose;
  RobotPose m_mapPose;
  std::unique_ptr<ShmemUtility> m_shmemUtil;

  void sendInitialPose(const std::shared_ptr<ros2_com::srv::SendInitialPose::Request> request,
          std::shared_ptr<ros2_com::srv::SendInitialPose::Response> response);

  void saveInitialPose(const std::shared_ptr<ros2_com::srv::SaveInitialPose::Request> request,
          std::shared_ptr<ros2_com::srv::SaveInitialPose::Response> response);

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

};
}
#endif //POSE_LISTENER_H