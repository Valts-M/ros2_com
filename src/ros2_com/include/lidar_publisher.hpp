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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  size_t count_;
  LidarLogReader log_reader_;
};
}
