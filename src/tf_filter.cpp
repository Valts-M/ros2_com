#include "tf_filter.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_com
{
  TfFilter::TfFilter() : rclcpp::Node("tf_filter")
  {
    m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    m_tfListener = this->create_subscription<tf2_msgs::msg::TFMessage>
    ("pre_filter_tf", 10, std::bind(&TfFilter::topicCallback, this, _1));
  }

  void TfFilter::topicCallback(tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    if(msg->transforms.back().header.frame_id != "map")
      m_tfBroadcaster->sendTransform(msg->transforms);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto tf_filter_node = std::make_shared<ros2_com::TfFilter>();
  rclcpp::spin(tf_filter_node);
  rclcpp::shutdown();
  return 0;
}