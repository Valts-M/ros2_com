#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include<iostream>
#include<fstream>

#include "rclcpp/rclcpp.hpp"
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;

class ClockServer : public rclcpp::Node
{
  public:
    ClockServer()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
      timer_ = this->create_wall_timer(
      10us, std::bind(&ClockServer::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      clock_.set__clock(this->now());
      publisher_->publish(clock_);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'.'%d'", clock_.clock.sec, clock_.clock.nanosec);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rosgraph_msgs::msg::Clock clock_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockServer>());
  rclcpp::shutdown();
  return 0;
}