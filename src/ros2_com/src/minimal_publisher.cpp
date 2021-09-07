#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0), tfBroad_(this)
    {
      tfMsg_.child_frame_id = "base_link";
      tfMsg_.header.frame_id = "odom";
      tfMsg_.transform.rotation.x = 0.0;
      tfMsg_.transform.rotation.y = 0.0;
      tfMsg_.transform.rotation.z = 0.0;
      tfMsg_.transform.rotation.w = 1.0;
      tfMsg_.transform.translation.x = 0.0;
      tfMsg_.transform.translation.y = 0.0;
      tfMsg_.transform.translation.z = 0.0;

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      tfMsg_.header.stamp = this->get_clock()->now();
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      tfBroad_.sendTransform(tfMsg_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    tf2_ros::TransformBroadcaster tfBroad_;
    geometry_msgs::msg::TransformStamped tfMsg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}