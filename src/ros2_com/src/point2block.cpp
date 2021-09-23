#include "point2block.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace ros2_com
{
Point2Block::Point2Block() : Node("point2block"), m_count(0)
{

  m_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&Point2Block::topicCallback, this, _1));
}

void Point2Block::topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  
}


}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::MapSaver>());
  rclcpp::shutdown();
  return 0;
}