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
Point2Block::Point2Block() : Node("point2block"), m_count(0), m_unfilteredCloud(new pcl::PointCloud<pcl::PointXYZ>)
{

  m_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&Point2Block::topicCallback, this, _1));
  m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
}

void Point2Block::topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, *m_unfilteredCloud);

  m_filter.setInputCloud(m_unfilteredCloud);
  m_filter.setFilterFieldName ("x");
  m_filter.setFilterLimits (-3.0, 3.0);
  m_filter.filter (*m_unfilteredCloud);
  m_filter.setFilterFieldName ("y");
  m_filter.filter(*m_unfilteredCloud);

  

  pcl::toROSMsg(*m_unfilteredCloud, m_filteredCloud);
  m_filteredCloud.header.frame_id = msg->header.frame_id;
  m_filteredCloud.header.stamp = this->now();
  m_publisher->publish(m_filteredCloud);
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::Point2Block>());
  rclcpp::shutdown();
  return 0;
}