#include "pc_filter.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
namespace ros2_com
{

PcFilter::PcFilter() : rclcpp::Node("pc_filter"),
  m_unfilteredCloud(new pcl::PointCloud<pcl::PointXYZ>),
  m_filteredCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  this->declare_parameter("length");
  this->declare_parameter("width");
  this->declare_parameter("height");
  this->declare_parameter("centerX");
  this->declare_parameter("centerY");
  this->declare_parameter("centerZ");

  const double length = this->get_parameter("length").as_double();
  const double width = this->get_parameter("width").as_double();
  const double height = this->get_parameter("height").as_double();
  const double transX = this->get_parameter("centerX").as_double();
  const double transY = this->get_parameter("centerY").as_double();
  const double transZ = this->get_parameter("centerZ").as_double();
  // RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f", minX, minY, minZ, transX, transY, transZ);

  m_boxFilter.setMin(Eigen::Vector4f(-width/2, -length/2, -height/2, 1.0));
  m_boxFilter.setMax(Eigen::Vector4f(width/2, length/2, height/2, 1.0));

  m_boxFilter.setTranslation(Eigen::Vector3f(transX, transY, transZ));
  m_boxFilter.setInputCloud(m_unfilteredCloud);
  m_boxFilter.setNegative(true);

  m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
  m_pcSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&PcFilter::pcTopicCallback, this, std::placeholders::_1));

}

void PcFilter::pcTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if(msg->data.empty())
    RCLCPP_ERROR(this->get_logger(), "EMPTY POINT CLOUD");

  pcl::fromROSMsg(*msg, *m_unfilteredCloud);

  m_boxFilter.filter(*m_filteredCloud);
  
  pcl::toROSMsg(*m_filteredCloud, m_filteredMsg);
  m_filteredMsg.header = msg->header;

  m_publisher->publish(m_filteredMsg);
}
} //namespace ros2_com

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::PcFilter>());
  rclcpp::shutdown();
  return 0;
}