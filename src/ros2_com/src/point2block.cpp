#include "point2block.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include <pcl/common/transforms.h>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace ros2_com
{
Point2Block::Point2Block() : Node("point2block"), m_count(0), 
m_unfilteredCloud(new pcl::PointCloud<pcl::PointXYZ>),
m_filteredCloud(new pcl::PointCloud<pcl::PointXYZ>),
m_rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  m_clearMap = cv::Mat(m_rows, m_cols, CV_8U);
  m_obstacleMap = cv::Mat(m_rows, m_cols, CV_8U);
  m_clearMap.setTo(127U);
  m_obstacleMap.setTo(0U);

  m_shmemUtil = std::make_unique<ShmemUtility>(std::vector<ConsProdNames>{ConsProdNames::p_LocalMap});
  m_shmemUtil->start();

  m_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&Point2Block::topicCallback, this, _1));

  m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

  m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
}

Point2Block::~Point2Block()
{
  m_shmemUtil->stop();
  m_shmemUtil.reset();
}

void Point2Block::topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  auto m_localMapProducer = m_shmemUtil->getShmem<PositionProducer>(ConsProdNames::p_MapPose);

  if(!m_localMapProducer) return;
  if(!m_localMapProducer->isObjectReferenced()) return;

  try 
  {
    m_mapLidarMsg = m_tfBuffer->lookupTransform(
      "odom", "base_footprint",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Could not transform map to laser_sensor_frame: %s", ex.what());
    return;
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "CALLBACK");

  geometry_msgs::msg::Quaternion* quat = &m_mapLidarMsg.transform.rotation;

  pcl::fromROSMsg(*msg, *m_unfilteredCloud);

  for(size_t i = 0; i < m_unfilteredCloud->points.size(); ++i)
  {
    pcl::PointXYZ point = m_unfilteredCloud->points.at(i);

    if(point.z < m_lidarTolerance && point.z > -m_lidarTolerance)
      m_filteredCloud->points.push_back(point);
    else
      if(std::abs(point.x) < 4.2426)
        if(std::abs(point.y) < 4.2426)
          m_filteredCloud->points.push_back(point);
  }

  pcl::transformPointCloud(*m_filteredCloud, *m_rotatedCloud, Eigen::Vector3f{}, 
    Eigen::Quaternionf(quat->w, quat->x, quat->y, quat->z));

  m_filteredCloud->clear();

  for(size_t i = 0; i < m_rotatedCloud->points.size(); ++i)
  {
    pcl::PointXYZ point = m_rotatedCloud->points.at(i);

    if(point.z < m_lidarTolerance && point.z > -m_lidarTolerance)
    {
      m_filteredCloud->points.push_back(point);
      updateClearImage(point);
    }
    else
      if(std::abs(point.x) < 3.0)
        if(std::abs(point.y) < 3.0)
        {
          m_filteredCloud->points.push_back(point);
          updateObstacleImage(point);
        }
  }

  pcl::toROSMsg(*m_filteredCloud, m_filteredCloudMsg);
  m_filteredCloudMsg.header.frame_id = msg->header.frame_id;
  m_filteredCloudMsg.header.stamp = this->now();
  m_publisher->publish(m_filteredCloudMsg);

  cv::imwrite("/workspaces/RobotV3/ros/src/ros2_com/obstacles.png", m_obstacleMap);
  cv::imwrite("/workspaces/RobotV3/ros/src/ros2_com/map.png", m_clearMap);
    
  m_clearMap.setTo(127);
  m_obstacleMap.setTo(0U);
  m_filteredCloud->clear();
  m_rotatedCloud->clear();
}

void Point2Block::updateClearImage(const pcl::PointXYZ& t_point)
{
    const cv::Point startPoint{m_rows / 2, m_cols / 2};
    const cv::Point endPoint{static_cast<int>(t_point.x * 100 + x_offset)/m_mapResolutionCm, 
      static_cast<int>(t_point.y * 100 + y_offset)/m_mapResolutionCm};
    cv::line(m_clearMap, startPoint, endPoint, 255U, 1);
    m_clearMap.at<unsigned char>(endPoint) = 0U;
}

void Point2Block::updateObstacleImage(const pcl::PointXYZ& t_point)
{
  if(t_point.z > -m_lidarHeight + m_floorTolerance || t_point.z < -m_lidarHeight - m_floorTolerance)
  {
    const cv::Point endPoint{static_cast<int>(t_point.x * 100 + x_offset) / m_mapResolutionCm, 
      static_cast<int>(t_point.y * 100 + y_offset) / m_mapResolutionCm};
    
    if(t_point.z <= -(m_lidarHeight + m_floorTolerance))
      m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::FALL;
    else if(t_point.z > -(m_lidarHeight + m_floorTolerance) && t_point.z <= -(m_lidarHeight - m_floorTolerance))
      m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::FLOOR;
    else if(t_point.z > -(m_lidarHeight - m_floorTolerance) && t_point.z <= -(m_robotHeight - m_lidarHeight + m_robotHeightTolerance))
      m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::OBSTACLE;
    else
      m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::TOO_HIGH;
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