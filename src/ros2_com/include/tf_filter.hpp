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

class TfFilter : public rclcpp::Node
{

public:
  TfFilter();

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tfListener{nullptr};

  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster{nullptr};

  void topicCallback(tf2_msgs::msg::TFMessage::SharedPtr msg);

};
}
#endif //TF_FILTER_H