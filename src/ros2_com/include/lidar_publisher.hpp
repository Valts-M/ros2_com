#ifndef LIDAR_PUBLISHER_H
#define LIDAR_PUBLISHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar_log_reader.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{
class LidarPublisher : public rclcpp::Node
{
public:
  LidarPublisher();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_{nullptr};
  size_t count_;
  LidarLogReader log_reader_;
};
}
#endif //LIDAR_PUBLISHER_H
