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
Point2Block::Point2Block() : Node("point2block"), m_count(0), 
m_unfilteredCloud(new pcl::PointCloud<pcl::PointXYZ>),
m_filteredCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  m_image = cv::Mat::zeros(120, 120, CV_8U);
  m_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&Point2Block::topicCallback, this, _1));
  m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
  m_imgPublisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
}

void Point2Block::topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, *m_unfilteredCloud);

  for(size_t i = 0; i < m_unfilteredCloud->points.size(); ++i)
  {
    if(std::abs(m_unfilteredCloud->points.at(i).x) < 3.0)
        if(std::abs(m_unfilteredCloud->points.at(i).y) <3)
          m_filteredCloud->points.push_back(m_unfilteredCloud->points.at(i));
  }

  // m_filter.setInputCloud(m_unfilteredCloud);
  // m_filter.setFilterFieldName ("x");
  // m_filter.setFilterLimits (-3.0, 3.0);
  // m_filter.filter (*m_unfilteredCloud);
  // m_filter.setFilterFieldName ("y");
  // m_filter.filter(*m_unfilteredCloud);

  pcl::toROSMsg(*m_filteredCloud, m_filteredCloudMsg);
  m_filteredCloudMsg.header.frame_id = msg->header.frame_id;
  m_filteredCloudMsg.header.stamp = this->now();
  m_publisher->publish(m_filteredCloudMsg);

  makeImage();
  // cv::imwrite("map.png", m_image);

  m_image = cv::Mat::zeros(120, 120, CV_8U);
  m_filteredCloud->clear();
}

void Point2Block::makeImage()
{
  RCLCPP_INFO(this->get_logger(), "points: %d, x: %f, idx: %d", 
  m_unfilteredCloud->points.size(), m_unfilteredCloud->points.at(20), static_cast<int>(m_unfilteredCloud->points.at(20).x));
  for(size_t i = 0; i < m_unfilteredCloud->points.size(); ++i)
  {
    int idx_x = static_cast<int>(m_unfilteredCloud->points.at(i).x * 100 + 300) % 5;
    int idx_y = static_cast<int>(m_unfilteredCloud->points.at(i).y * 100 + 300) % 5;
    //RCLCPP_INFO(this->get_logger(), "%d, %d", idx_x, idx_y);
    m_image.at<unsigned char>(idx_x, idx_y) = 255;
  }
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::Point2Block>());
  rclcpp::shutdown();
  return 0;
}