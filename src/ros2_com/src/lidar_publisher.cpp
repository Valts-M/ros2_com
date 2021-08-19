#include "lidar_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace ros2_com
{
LidarPublisher::LidarPublisher() : Node("lidar_publisher"), count_(0)
{
  publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  timer_ = this->create_wall_timer(
    100ms, std::bind(&LidarPublisher::timer_callback, this));
}

void LidarPublisher::timer_callback()
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
  rclcpp::spin(std::make_shared<ros2_com::LidarPublisher>());
  rclcpp::shutdown();
  return 0;
}
