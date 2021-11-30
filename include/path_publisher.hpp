#ifndef PATH_PUBLISHER_H
#define PATH_PUBLISHER_H

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
#include "nav_msgs/msg/path.hpp"

namespace ros2_com
{

struct PathTracker
{
  bool enabled{false};
  std::string parent_frame;
  std::string child_frame;
  std::string param_name;
  geometry_msgs::msg::TransformStamped currTf;
  geometry_msgs::msg::TransformStamped prevTf;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher{nullptr};
  nav_msgs::msg::Path path;
};


inline geometry_msgs::msg::Vector3 operator-(const geometry_msgs::msg::Vector3& a, const geometry_msgs::msg::Vector3& b)
{
  geometry_msgs::msg::Vector3 temp;
  temp.x = a.x - b.x;
  temp.y = a.y - b.y;
  temp.z = a.z - b.z;
  return temp;
}

inline bool operator>(const geometry_msgs::msg::Vector3& a, const double b)
{
  const double tmp = cbrt(a.x * a.x + a.y * a.y + a.z * a.z);
  return tmp > b;
}

/**
 * @brief Node that listens for the map -> laser_sensor_frame and odom -> laser_sensor_frame
 * transforms and sends them over Shmem to RobotV3
 * 
 */
class PathPublisher : public rclcpp::Node
{

public:
/**
 * @brief Construct a new Pose Listener object
 * 
 */
  PathPublisher();

  enum alias
  {
    robotMap,
    robotOdom,
    lidarMap,
    lidarOdom,
    count
  };

  PathTracker m_paths[alias::count];


private:

  static constexpr double m_pathUpdateDist{0.2};

  /*!
    * @brief Updates the travelled path in the odom frame
  */
  void updatePaths();

  bool initTf();

  /*!
    * @brief Updates the travelled path in the odom frame
  */
  void updatePath(int i);

  void updateTransforms();

  void updateTransform();

  void updateParams();

  geometry_msgs::msg::PoseStamped toPose(geometry_msgs::msg::TransformStamped& tf);

  /**
   * @brief Ros timer callback
   * 
   */
  void timerCallback();

  /*!
    * @brief Transform listener
  */
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};

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
   * @brief string with the lidar frame name
   * 
   */
  std::string m_lidar_frame;

  /**
   * @brief string with the lidar frame name
   * 
   */
  std::string m_base_frame;

};
}
#endif //PATH_PUBLISHER_H