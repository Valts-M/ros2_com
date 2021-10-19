#ifndef POINT_2_BLOCK_H
#define POINT_2_BLOCK_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>

//robotv3
#include "shmem_util.hpp"
#include "data_structures/locald_data.hpp"

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <opencv2/opencv.hpp>

namespace ros2_com
{

using namespace zbot;

enum ProjectionTypes :uint8_t {
    UNKNOWN = 0U,
    FALL = 1U << 0U,
    FLOOR = 1U << 1U,
    OBSTACLE = 1U << 2U,
    TOO_HIGH = 1U << 3U
};

class Point2Block : public rclcpp::Node
{

public:
  Point2Block();
  ~Point2Block();

private:

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSubscriber{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_lsSubscriber{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher{nullptr};
  sensor_msgs::msg::PointCloud2 m_filteredCloudMsg;
  sensor_msgs::msg::LaserScan::SharedPtr m_laserScan;
  std::unique_ptr<ShmemUtility> m_shmemUtil;

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_unfilteredCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_rotatedCloud;
  
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
  geometry_msgs::msg::TransformStamped m_mapLidarMsg;
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

  cv::Mat m_clearMap;
  cv::Mat m_obstacleMap;

  const double m_lidarHeight{0.345};
  const double m_robotHeight{1.2};
  const double m_lidarTolerance{0.1};
  const double m_floorTolerance{0.07};
  const double m_robotHeightTolerance{0.1};

  const u_int16_t m_rows{120};
  const u_int16_t m_cols{120};
  const int m_mapResolutionCm{5};
  const int x_offset{m_rows * m_mapResolutionCm / 2};
  const int y_offset{m_cols * m_mapResolutionCm / 2};
  const float m_lidarBlindRadius = 0.5;

  void pcTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lsTopicCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void makeClearImage();
  void updateObstacleImage(const pcl::PointXYZ& t_point);


  /*!
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

};
}
#endif //POINT_2_BLOCK_H