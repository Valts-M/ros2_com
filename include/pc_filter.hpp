#ifndef PC_FILTERS_H
#define PC_FILTERS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

namespace ros2_com
{

class PcFilter : public rclcpp::Node
{
  public:
    PcFilter();

  private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSubscriber{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher{nullptr};
    sensor_msgs::msg::PointCloud2 m_filteredMsg{};

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_unfilteredCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud;

  pcl::CropBox<pcl::PointXYZ> m_boxFilter{};
    void pcTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
};
}


#endif