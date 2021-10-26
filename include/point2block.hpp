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

  void initParams();

  double m_lidarHeight;
  double m_robotHeight;
  double m_floorTolerance;
  double m_robotHeightTolerance;

  int m_rows;
  int m_cols;
  int m_mapResolutionCm;
  double m_lidarBlindRadius;
  int x_offset;
  int y_offset;

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