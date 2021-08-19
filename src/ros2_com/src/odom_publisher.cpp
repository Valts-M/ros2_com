#include "odom_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{
    OdometryPublisher::OdometryPublisher()
    : Node("odom_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&OdometryPublisher::timer_callback, this));
    }

    void OdometryPublisher::timer_callback()
    {
      // auto message = std_msgs::msg::String();
      // message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // publisher_->publish(message);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
