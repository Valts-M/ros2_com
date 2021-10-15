#ifndef TF_FILTER_H
#define TF_FILTER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace ros2_com
{

/**
 * @brief Node for filtering out map -> odom transformations so that we can run slam on recordings
 * where slam was active
 * 
 */
class TfFilter : public rclcpp::Node
{

public:
  /**
   * @brief Construct a new Tf Filter object
   * 
   */
  TfFilter();

private:

  /**
   * @brief Shared pointer to a TFMEssage subscriber
   * 
   */
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tfListener{nullptr};

  /**
   * @brief Transform broadcaster that publishes the filtered transforms
   * 
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster{nullptr};

  /**
   * @brief Subscription topic callback
   * 
   * @param msg 
   */
  void topicCallback(tf2_msgs::msg::TFMessage::SharedPtr msg);

};
}
#endif //TF_FILTER_H